#!/usr/bin/env python3
"""
Renkli küp algılayıcı (RGB-D + HSV segmentasyon).

İşlem hattı (her kare için):
  1. RGB görüntü HSV uzayına çevrilir. HSV'de renk tonu (H) ışık şiddetinden (V)
     ayrıldığı için gölge/aydınlatma değişimlerine RGB'den çok daha dayanıklıdır.
  2. Her renk için H aralığı + yüksek doygunluk (S) eşiğiyle ikili maske çıkarılır.
     Kırmızı, H çemberinin iki ucunda (0 ve 180) olduğu için iki aralık birleştirilir.
  3. Morfolojik açma (erode+dilate) ile gürültü pikselleri silinir.
  4. Bağlantılı bileşenler (connected components) ile her blob ayrı ele alınır.
  5. Blobun piksellerindeki MEDYAN derinlik alınır (kenar piksellerinin hatalı
     derinliğine karşı sağlam).
  6. Pinhole kamera modeli ile 3B nokta:
        X = (u - cx) * Z / fx ,  Y = (v - cy) * Z / fy ,  Z = derinlik
  7. TF2 ile nokta 'map' ve 'base_footprint' çerçevelerine dönüştürülür.
  8. Fiziksel boyutu küp için fazla büyük olanlar (renkli kutular) ve yerden
     yüksekte olanlar (gripper'da tutulan küp) elenir.

Yayınlar:
  /mrm/detections          mrm_interfaces/DetectionArray
  /mrm/detection_markers   visualization_msgs/MarkerArray  (RViz)
  /mrm/detection_image     sensor_msgs/Image  (etiketli hata ayıklama görüntüsü)
"""

import math
import threading

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from mrm_interfaces.msg import Detection, DetectionArray
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_point  # noqa: F401  (PointStamped desteğini kaydeder)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# (H_min, H_max) listesi - OpenCV'de H: 0..179
HSV_RANGES = {
    'red':   [(0, 8), (170, 179)],
    'green': [(45, 85)],
    'blue':  [(105, 130)],
}
MARKER_RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.1), 'blue': (0.1, 0.3, 1.0)}
DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        self.declare_parameter('min_saturation', 120)
        self.declare_parameter('min_value', 40)
        self.declare_parameter('min_area_px', 12)
        self.declare_parameter('max_object_size', 0.09)   # m; bundan büyük blob = kutu
        # Yerdeki küpün görünen yüzeyi en fazla ~3 cm yüksekte. Daha yüksek bloblar: taşınan küp
        # (~0.27 m) veya kutu duvarı parçası (AprilTag ön yüzü iki dar renkli şeride böler, ~5 cm)
        self.declare_parameter('max_object_height', 0.045)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('object_size', 0.035)       # küp kenarı (yüzey->merkez düzeltmesi)

        self.bridge = CvBridge()
        self.K = None
        self.tf_buffer = Buffer()
        # TF dinleyicisi ayrı bir düğümde, kendi thread'inde döner. Aksi halde görüntü
        # callback'i TF'i (zaman aşımıyla) beklerken TF mesajlarını işleyecek thread
        # de bloke olur ve algılayıcı kendi beklediği veriyi alamaz.
        self.tf_node = rclpy.create_node(
            'color_detector_tf',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node)
        self.tf_exec = SingleThreadedExecutor()
        self.tf_exec.add_node(self.tf_node)
        threading.Thread(target=self.tf_exec.spin, daemon=True).start()

        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info', self.on_info, 1)
        rgb = message_filters.Subscriber(self, Image, '/camera/rgbd/image_raw')
        depth = message_filters.Subscriber(self, Image, '/camera/rgbd/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 5, 0.05)
        self.sync.registerCallback(self.on_images)

        self.pub = self.create_publisher(DetectionArray, '/mrm/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mrm/detection_markers', 10)
        self.dbg_pub = self.create_publisher(Image, '/mrm/detection_image', 2)
        # Nav2 costmap'i için: lidar 3.5 cm'lik küpleri göremez (0.13 m yükseklikte tarar).
        # Kameranın gördüğü küpler küçük bir nokta bulutu olarak yayınlanır; Nav2 engel
        # katmanları bunu bir sensör kaynağı gibi kullanır -> robot küplere çarpmaz ve onları itmez.
        self.obst_pub = self.create_publisher(PointCloud2, '/mrm/piece_obstacles', 5)
        self.get_logger().info('Renk algilayici hazir')

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def transform_all(self, p_cam, header):
        """
        Kamera noktası -> (odom, base_footprint, map) noktaları; hata olursa None.

        1) Görüntünün çekildiği ANDAKİ dönüşümle 'odom' çerçevesine çevir.
           odom->base TF'i tekerlek odometrisinden yüksek frekansta gelir: her zaman taze.
           (Robot dönerken "en güncel" TF'i kullanmak eski kareyi yeni pozla eşleştirir.)
        2) base_footprint: aynı (görüntü) anındaki robot pozuna göre.
        3) map: map->odom (AMCL) yavaş değişir ve birkaç saniye geriden gelebilir:
           en güncel tahmini kullan.
        """
        ps = PointStamped()
        # Not: header kopyalanmalıdır (aynı nesneyi paylaşmak gelen mesajı değiştirir)
        ps.header.frame_id = header.frame_id
        ps.header.stamp = header.stamp
        ps.point.x, ps.point.y, ps.point.z = map(float, p_cam)
        try:
            p_odom = self.tf_buffer.transform(ps, 'odom', timeout=Duration(seconds=0.2))
            p_base = self.tf_buffer.transform(p_odom, 'base_footprint', timeout=Duration(seconds=0.2))
            p_odom_latest = PointStamped()
            p_odom_latest.header.frame_id = 'odom'
            p_odom_latest.header.stamp = rclpy.time.Time().to_msg()
            p_odom_latest.point = p_odom.point
            p_map = self.tf_buffer.transform(p_odom_latest, 'map', timeout=Duration(seconds=0.2))
            return p_odom.point, p_base.point, p_map.point
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF hatasi: {e}', throttle_duration_sec=5.0)
            return None

    def on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        smin = self.get_parameter('min_saturation').value
        vmin = self.get_parameter('min_value').value
        min_area = self.get_parameter('min_area_px').value
        max_size = self.get_parameter('max_object_size').value
        max_h = self.get_parameter('max_object_height').value
        max_range = self.get_parameter('max_range').value
        kernel = np.ones((3, 3), np.uint8)

        out = DetectionArray()
        out.header.stamp = rgb_msg.header.stamp
        out.header.frame_id = 'map'
        dbg = bgr.copy() if self.get_parameter('publish_debug_image').value else None
        cam = None          # kamera merkezinin (odom, base, map) konumu: kare başına bir kez

        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for hmin, hmax in ranges:
                mask |= cv2.inRange(hsv, (hmin, smin, vmin), (hmax, 255, 255))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
            for i in range(1, n):
                area = stats[i, cv2.CC_STAT_AREA]
                if area < min_area:
                    continue
                d = depth[labels == i]
                d = d[np.isfinite(d) & (d > 0.05)]
                if d.size < 5:
                    continue
                z = float(np.median(d))
                if z > max_range:
                    continue
                # Blobun yaklaşık fiziksel boyutu (en geniş kenar)
                w_px = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
                size_m = w_px * z / fx
                if size_m > max_size:
                    continue  # Renkli kutu ya da büyük yüzey
                # Görüntü kenarına değen bloblar kesik olabilir -> ele
                x0, y0 = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
                if x0 <= 1 or y0 <= 1 or x0 + stats[i, cv2.CC_STAT_WIDTH] >= bgr.shape[1] - 1 \
                        or y0 + stats[i, cv2.CC_STAT_HEIGHT] >= bgr.shape[0] - 1:
                    continue  # (alt kenar dahil: yakındaki küp alttan kesilince merkezi uzakta çıkar)
                u, v = cents[i]
                p_cam = ((u - cx) * z / fx, (v - cy) * z / fy, z)
                pts = self.transform_all(p_cam, rgb_msg.header)
                if pts is None:
                    continue
                po, pb, pm = pts
                if pm.z > max_h:
                    continue
                # Kamera küpün ÖN YÜZÜNÜ görür; merkez, bakış yönünde yarım kenar ileridedir.
                # Düzeltmeyi yatay düzlemde, kameradan noktaya doğru uygula.
                # Kalibrasyonla ölçüldü: tam yarım kenar (1.75 cm) düzeltmesi 0.2-0.75 m'de
                # sistematik +5 mm fazla veriyordu (kamera ön yüz + üst yüzün bir kısmını görür).
                half = self.get_parameter('object_size').value / 2.0 - 0.005
                if cam is None:
                    cam = self.transform_all((0.0, 0.0, 0.0), rgb_msg.header)
                if cam is None:
                    continue
                for p, c in zip((po, pb, pm), cam):
                    dx, dy = p.x - c.x, p.y - c.y
                    n = math.hypot(dx, dy)
                    if n > 1e-3:
                        p.x += half * dx / n
                        p.y += half * dy / n
                det = Detection()
                det.color = color
                det.position_map = pm
                det.position_base = pb
                det.position_odom = po
                det.pixel_area = float(area)
                det.depth = z
                out.detections.append(det)
                if dbg is not None:
                    bx, by, bw, bh = x0, y0, stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
                    cv2.rectangle(dbg, (bx - 2, by - 2), (bx + bw + 2, by + bh + 2), DRAW_BGR[color], 2)
                    cv2.putText(dbg, f'{color} {z:.2f}m', (bx, max(10, by - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, DRAW_BGR[color], 1)

        self.pub.publish(out)
        self.publish_markers(out)
        self.publish_obstacles(out)
        if dbg is not None:
            msg = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            msg.header.frame_id = rgb_msg.header.frame_id
            msg.header.stamp = rgb_msg.header.stamp
            self.dbg_pub.publish(msg)

    def publish_obstacles(self, arr):
        pts = []
        for d in arr.detections:
            for dx in (-0.02, 0.0, 0.02):
                for dy in (-0.02, 0.0, 0.02):
                    pts.append((d.position_map.x + dx, d.position_map.y + dy, 0.03))
        hdr = arr.header  # frame: map
        self.obst_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

    def publish_markers(self, arr):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, d in enumerate(arr.detections):
            m = Marker()
            m.header.frame_id = arr.header.frame_id
            m.header.stamp = arr.header.stamp
            m.ns, m.id, m.type, m.action = 'detections', i, Marker.SPHERE, Marker.ADD
            m.pose.position = d.position_map
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b = MARKER_RGB[d.color]
            m.color.a = 0.8
            m.lifetime = Duration(seconds=0.5).to_msg()
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(ColorDetector())


if __name__ == '__main__':
    main()
