#!/usr/bin/env python3
"""
BECERİ SUNUCUSU (Skill Server)
==============================

PlanSys2 "ne yapılacağını" (pick red_room1_0 ...) söyler; bu düğüm "nasıl
yapılacağını" bilir. Her beceri; Nav2, MoveIt, kamera, gripper ve tabanın
hassas hareketlerini sıralı bir senaryoda birleştirir.

Arayüz:
  Action  /mrm/execute_skill   (mrm_interfaces/action/ExecuteSkill)
          skill: navigate_room | scan_room | pick | place | open_door | charge | home_arm
  Service /mrm/list_pieces     (mrm_interfaces/srv/ListPieces) -> dünya modelindeki parçalar

Neden ayrı bir düğüm?
  * Beceriler PlanSys2 olmadan da tek tek test edilebilir:
      ros2 action send_goal /mrm/execute_skill mrm_interfaces/action/ExecuteSkill \
        "{skill: scan_room, args: [room1]}" --feedback
  * PlanSys2 action düğümleri ince kalır (sadece çeviri yapar).

DÜNYA MODELİ: Parçalar kamerayla bulunur (scan_room) ve burada kayıt altına
alınır. Kutuların, kapıların, dock'un yerleri bilinen harita bilgisidir (world.yaml).
"""

import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from mrm_interfaces.action import ExecuteSkill
from mrm_interfaces.msg import DetectionArray, DoorState, Piece
from mrm_interfaces.srv import ListPieces
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CameraInfo, LaserScan
import cv2
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from mrm_skills import arm_ik
from mrm_skills.piece_tracker import PieceTracker, corridor_blocked, pose_conflicts, to_local
from mrm_skills.robot_api import Attacher, BaseMotion, MoveItApi, Nav2Api, norm_angle, quat_to_yaw

GRIPPER_OPEN = 0.019
GRIPPER_CLOSED = -0.010


class SkillError(Exception):
    pass


class SkillServer(Node):

    def __init__(self):
        super().__init__('skill_server')
        cfg = os.path.join(get_package_share_directory('mrm_simulation'), 'config', 'world.yaml')
        self.declare_parameter('world_config', cfg)
        self.declare_parameter('gripper_grasp', -0.002)     # Küpü kavrarken parmak konumu
        self.declare_parameter('pick_target_x', 0.17)       # Küp bu mesafeye (base) getirilir
        self.declare_parameter('place_target_x', 0.24)      # Kutu merkezi bu mesafeye getirilir
        self.declare_parameter('press_target_x', 0.27)      # Buton merkezi bu mesafeye getirilir
        self.declare_parameter('min_observations', 3)       # Bir parçanın kabulü için min görülme
        self.declare_parameter('approach_v_max', 0.12)      # Görsel servo yanaşmada en yüksek hız (m/s)
        self.declare_parameter('grasp_x', 0.21)             # Kavrama anında küpün base x'i
        self.scan_count = {}                                 # oda -> kaçıncı tarama
        # Yalnızca teşhis: kavrama anında küplerin gerçek konumunu (Gazebo) robot çerçevesinde
        # logla. Kontrol için kullanılmaz; algılama hatasını ölçmek içindir.
        self.declare_parameter('debug_ground_truth', False)
        with open(self.get_parameter('world_config').value) as f:
            self.world = yaml.safe_load(f)

        self.cb = ReentrantCallbackGroup()
        # TF ve odometri dinleyicileri AYRI düğümlerde, AYRI bir thread'de: beceri
        # thread'leri ve algı mesajları ne kadar yük bindirirse bindirsin pozlar taze kalır.
        self.tf_buffer = Buffer()
        self.tf_node = rclpy.create_node(
            'skill_server_tf',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node)
        self.aux_executor = MultiThreadedExecutor(num_threads=2)
        self.aux_executor.add_node(self.tf_node)

        # Algılar
        self.detections = []         # (zaman, DetectionArray) kayan pencere
        self.det_lock = threading.Lock()
        self.doors = {}
        self.battery = None
        self.scan = None
        self.create_subscription(DetectionArray, '/mrm/detections', self.on_det, 10, callback_group=self.cb)
        self.create_subscription(DoorState, '/mrm/doors', self.on_door, 10, callback_group=self.cb)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10, callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 5, callback_group=self.cb)
        # AprilTag algıları (apriltag_ros): hangi etiket ne zaman görüldü
        self.tag_seen = []          # (sim_zaman, id, header.stamp)
        self.tag_lock = threading.Lock()
        self.create_subscription(AprilTagDetectionArray, '/apriltag/detections', self.on_tags, 10,
                                 callback_group=self.cb)
        self.cam_K = None
        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info', self.on_cam_info, 1,
                                 callback_group=self.cb)

        # Hedef küp takipçisi (odom çerçevesinde, her karede güncellenir)
        self.tracker = PieceTracker()
        # Dünya modeli: isim -> dict(color, room, x, y, obs, state)  state: free|held|placed|lost
        self.pieces = {}
        self.held_model = None       # Gazebo'da tutulan modelin adı (detach için)

        # Alt sistemler
        self.nav = Nav2Api(self.get_logger())
        self.moveit = MoveItApi(self, self.cb)
        self.base = BaseMotion(self, self.tf_buffer, self.aux_executor)
        threading.Thread(target=self.aux_executor.spin, daemon=True).start()
        self.attacher = Attacher(self, self.cb)

        self.gt_models = None
        if self.get_parameter('debug_ground_truth').value:
            from gazebo_msgs.msg import ModelStates
            self.create_subscription(ModelStates, '/gazebo/model_states',
                                     lambda m: setattr(self, 'gt_models', m), 1, callback_group=self.cb)
            from sensor_msgs.msg import JointState
            self.gt_joints = None
            self.create_subscription(JointState, '/joint_states',
                                     lambda m: setattr(self, 'gt_joints', m), 1, callback_group=self.cb)
        self.right_finger_pub = self.create_publisher(
            Float64MultiArray, '/gripper_right_controller/commands', 10)
        self.create_service(ListPieces, '/mrm/list_pieces', self.on_list_pieces, callback_group=self.cb)
        # Dünya modelindeki YERDEKİ küpler -> kalıcı engel bulutu (Nav2 costmap 'known_pieces' kaynağı).
        # Kamera robotun dibindeki küpü göremez ve costmap temizlenince canlı algı izleri de silinir;
        # bu bulut, bilinen küplerin her zaman engel olarak kalmasını sağlar (robot onları itmez).
        self.known_pub = self.create_publisher(PointCloud2, '/mrm/known_pieces', 5)
        self.create_timer(0.5, self.publish_known_pieces, callback_group=self.cb)
        self.busy = False
        self.goal_handle = None
        self.server = ActionServer(
            self, ExecuteSkill, '/mrm/execute_skill', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb)
        self.skills = {
            'navigate_room': self.skill_navigate_room,
            'scan_room': self.skill_scan_room,
            'pick': self.skill_pick,
            'place': self.skill_place,
            'open_door': self.skill_open_door,
            'charge': self.skill_charge,
            'home_arm': self.skill_home_arm,
            # Yalnızca test/teşhis (alt simülasyonlar, mrm_skills/skill_test.py)
            'debug_reset': self.skill_debug_reset,
            'debug_add_piece': self.skill_debug_add_piece,
        }
        self.get_logger().info('Beceri sunucusu hazir: /mrm/execute_skill')

    # ------------------------------------------------------------------ algılar
    def on_det(self, msg):
        # Zaman = görüntünün SİM zaman damgası (geliş zamanı değil): algılama
        # gecikmeli işlense bile hangi ana ait olduğu doğru bilinir.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.det_lock:
            self.detections.append((stamp, msg))
            self.detections = self.detections[-400:]
        if self.tracker.active:
            self.tracker.update(stamp, [(d.color, d.position_odom.x, d.position_odom.y,
                                         d.position_base.x) for d in msg.detections])

    def now_sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, seconds):
        """Sim zamanında bekle (gerçek zaman faktörü < 1 olabilir)."""
        t_end = self.now_sim() + seconds
        while self.now_sim() < t_end:
            time.sleep(0.05)

    def on_door(self, msg):
        self.doors[msg.name] = msg

    def on_battery(self, msg):
        self.battery = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_tags(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.tag_lock:
            for d in msg.detections:
                corners = np.array([[c.x, c.y] for c in d.corners], dtype=np.float64)
                self.tag_seen.append((t, d.id, corners))
            self.tag_seen = self.tag_seen[-300:]

    def on_cam_info(self, msg):
        self.cam_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def tag_size(self, tag_id):
        if tag_id == self.world['dock']['tag_id']:
            return self.world['dock']['tag_size']
        for b in self.world['bins'].values():
            if b['tag_id'] == tag_id:
                return b['tag_size']
        raise KeyError(tag_id)

    def cam_to_base(self):
        """Kamera optik çerçevesi -> base_footprint: STATİK dönüşüm (zamanlama sorunu yok)."""
        if getattr(self, '_T_bc', None) is None:
            tr = self.tf_buffer.lookup_transform('base_footprint', 'rgbd_camera_optical_frame',
                                                 rclpy.time.Time())
            q = tr.transform.rotation
            R = np.array([
                [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.x * q.w)],
                [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x ** 2 + q.y ** 2)]])
            t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                          tr.transform.translation.z])
            self._T_bc = (R, t)
        return self._T_bc

    def recent_detections(self, since, until=None):
        """Görüntü zaman damgası [since, until] aralığında olan algılar."""
        with self.det_lock:
            return [d for t, arr in self.detections
                    if t >= since and (until is None or t <= until) for d in arr.detections]

    def publish_known_pieces(self):
        pts = []
        for p in self.pieces.values():
            if p['state'] != 'free':
                continue
            for dx in (-0.02, 0.0, 0.02):
                for dy in (-0.02, 0.0, 0.02):
                    pts.append((p['x'] + dx, p['y'] + dy, 0.03))
        hdr = Header()
        hdr.frame_id = 'map'
        hdr.stamp = self.get_clock().now().to_msg()
        self.known_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

    def on_list_pieces(self, req, res):
        for name, p in self.pieces.items():
            if req.room and p['room'] != req.room:
                continue
            if p['state'] != 'free':
                continue
            m = Piece()
            m.name, m.color, m.room = name, p['color'], p['room']
            m.position.x, m.position.y = p['x'], p['y']
            m.observations = p['obs']
            res.pieces.append(m)
        return res

    # ------------------------------------------------------------- action akışı
    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warn('Yeni hedef reddedildi: sunucu mesgul')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        self.busy = True
        self.goal_handle = goal_handle
        skill, args = goal_handle.request.skill, list(goal_handle.request.args)
        self.get_logger().info(f'>>> BECERI {skill} {args}')
        result = ExecuteSkill.Result()
        try:
            if skill not in self.skills:
                raise SkillError(f'bilinmeyen beceri: {skill}')
            # Lokalizasyon (AMCL map->odom) hazır olana kadar bekle
            t0 = time.monotonic()
            while not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                if time.monotonic() - t0 > 30.0:
                    raise SkillError('map -> base_footprint TF yok (AMCL calisiyor mu?)')
                time.sleep(0.5)
            msg = self.skills[skill](*args)
            result.success, result.message = True, msg or 'ok'
        except SkillError as e:
            result.success, result.message = False, str(e)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Beklenmeyen hata: {e!r}')
            result.success, result.message = False, f'exception: {e!r}'
        finally:
            self.base.stop()
            self.busy = False
        self.get_logger().info(f'<<< {skill}: {"BASARILI" if result.success else "BASARISIZ"} - {result.message}')
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def cancelled(self):
        return self.goal_handle is not None and self.goal_handle.is_cancel_requested

    def fb(self, progress, status):
        if self.goal_handle is not None:
            f = ExecuteSkill.Feedback()
            f.progress, f.status = float(progress), status
            self.goal_handle.publish_feedback(f)
        self.get_logger().info(f'   [{progress*100:3.0f}%] {status}')

    def check_cancel(self):
        if self.cancelled():
            raise SkillError('iptal edildi')

    # ------------------------------------------------------------ yardımcılar
    def map_pose(self):
        return self.base.pose('map')

    def to_base(self, x, y):
        """map noktasını base_footprint çerçevesine çevir (2B)."""
        rx, ry, ryaw = self.map_pose()
        dx, dy = x - rx, y - ry
        c, s = math.cos(ryaw), math.sin(ryaw)
        return c * dx + s * dy, -s * dx + c * dy

    def go(self, x, y, yaw, retries=2):
        for k in range(retries + 1):
            self.check_cancel()
            if self.nav.go_to(x, y, yaw, cancel_check=self.cancelled,
                              feedback=lambda s: self.get_logger().info(s, throttle_duration_sec=3.0)):
                return
            self.get_logger().warn(f'Nav2 denemesi {k+1} basarisiz, costmap temizleniyor')
            self.nav.clear_costmaps()
            time.sleep(1.0)
        raise SkillError(f'Nav2 hedefe ulasamadi ({x:.2f}, {y:.2f})')

    def arm_to(self, q, what='kol', vel=0.5, required=True):
        """MoveIt ile eklem hedefi. Tork kontrolünde geçici tolerans aşımı olabilir: 1 kez tekrar dene."""
        for attempt in range(2):
            self.check_cancel()
            if self.moveit.arm(q, vel):
                return True
            self.get_logger().warn(f'MoveIt {what} hareketi basarisiz (deneme {attempt + 1})')
            time.sleep(0.5)
        if required:
            raise SkillError(f'MoveIt {what} hareketi basarisiz')
        return False

    def arm_ik(self, x, y, z, psis, what):
        q, psi = arm_ik.ik_search(x, y, z, psis)
        if q is None:
            raise SkillError(f'{what}: IK cozumu yok ({x:.3f}, {y:.3f}, {z:.3f})')
        return q, psi

    def gripper(self, pos):
        # Sağ parmak (mimic çalışmadığı için) ayrı kontrolcüye aynı değer
        self.right_finger_pub.publish(Float64MultiArray(data=[float(pos)]))
        if not self.moveit.gripper(pos):
            self.get_logger().warn('Gripper hareketi raporu basarisiz (devam)')

    def face_point(self, x, y):
        """Yerinde dönerek map'teki (x, y) noktasına bak."""
        bx, by = self.to_base(x, y)
        self.base.rotate(math.atan2(by, bx))

    def room_of(self, x, y):
        for name, r in self.world['rooms'].items():
            x0, x1, y0, y1 = r['bounds']
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    def in_bin_zone(self, x, y):
        """
        Kutu çerçevesinde dikdörtgen bölge: yanal ±0.17 m, kutunun arkasından (-0.16 m) ön yüzün
        0.15 m önüne (+0.26 m) kadar. AprilTag ön yüzü iki dar renkli şeride böler; algı hatasıyla
        bu şeritler yüzün birkaç cm önünde ve yüz genişliği boyunca "küp" gibi görünüyordu.
        (Daire şeklindeki eski bölge yüz kenarlarındaki şeritleri kaçırıyordu.)
        """
        for b in self.world['bins'].values():
            dx, dy = x - b['position'][0], y - b['position'][1]
            c, s_ = math.cos(b['approach_yaw']), math.sin(b['approach_yaw'])
            n = -(dx * c + dy * s_)        # + : kutunun önü (robotun geldiği taraf)
            lat = -dx * s_ + dy * c
            if abs(lat) <= 0.17 and -0.16 <= n <= 0.26:
                return True
        return False

    def near_bin(self, x, y, dist=0.28):
        return any(math.hypot(x - b['position'][0], y - b['position'][1]) < dist
                   for b in self.world['bins'].values())

    def log_ground_truth(self, bx, by, what):
        """Teşhis: hedef (bx, by) ile robot çerçevesindeki en yakın gerçek küp arasındaki fark."""
        m = self.gt_models
        if m is None or 'mrm_robot' not in m.name:
            return
        r = m.pose[m.name.index('mrm_robot')]
        ryaw = math.atan2(2 * (r.orientation.w * r.orientation.z), 1 - 2 * r.orientation.z ** 2)
        best = None
        for n, p in zip(m.name, m.pose):
            if not n.startswith('piece_'):
                continue
            dx, dy = p.position.x - r.position.x, p.position.y - r.position.y
            tx = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            ty = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            d = math.hypot(tx - bx, ty - by)
            if best is None or d < best[0]:
                best = (d, n, tx, ty)
        js = getattr(self, 'gt_joints', None)
        jtxt = ''
        if js is not None:
            jtxt = ' kol=' + ','.join(f'{v:.2f}' for n, v in zip(js.name, js.position) if n.startswith('joint'))
        if best:
            self.get_logger().warn(f'[GT {what}] hedef=({bx:.3f},{by:.3f}) gercek {best[1]}=({best[2]:.3f},'
                                   f'{best[3]:.3f}) hata dx={best[2]-bx:+.3f} dy={best[3]-by:+.3f}{jtxt}')

    # ================================================================= BECERİLER
    def skill_home_arm(self):
        self.arm_to(arm_ik.HOME, 'home')
        return 'kol home pozunda'

    def scan_viewpoints(self, room):
        """Oda merkezi + çapraz noktalar; kutulara, kapı/buton önüne, dock'a yakın olanlar elenir."""
        cx, cy = self.world['rooms'][room]['center']
        keep = self.world['rooms'][room]['keepout'][1:]     # [0] = merkezin kendisi
        vps = [(cx, cy)]
        for dx, dy in ((0.9, 0.9), (-0.9, -0.9), (0.9, -0.9), (-0.9, 0.9)):
            px, py = cx + dx, cy + dy
            if all(math.hypot(px - kx, py - ky) > kr for kx, ky, kr in keep):
                vps.append((px, py))
        return vps

    def skill_navigate_room(self, room, *_):
        """Odanın merkezine Nav2 ile git (kapılar açık olmalı)."""
        cx, cy = self.world['rooms'][room]['center']
        rx, ry, _ = self.map_pose()
        yaw = math.atan2(cy - ry, cx - rx)
        self.fb(0.1, f'{room} merkezine gidiliyor')
        self.go(cx, cy, yaw)
        return f'{room} merkezine ulasildi'

    def skill_scan_room(self, room, *_):
        """
        Oda merkezinde 360° dönerek renkli küpleri ara ve dünya modeline kaydet.
        Aynı renkte ve 15 cm'den yakın algılar tek parça olarak kümelenir;
        en az min_observations kez görülen kümeler parça kabul edilir.
        """
        x0, x1, y0, y1 = self.world['rooms'][room]['bounds']
        k = self.scan_count.get(room, 0)
        self.scan_count[room] = k + 1
        vps = self.scan_viewpoints(room)
        # 1. tarama: merkez + ilk çapraz nokta; sonraki (doğrulama) taramaları: sıradaki noktalar
        chosen = vps[:2] if k == 0 else [vps[(1 + k + j) % len(vps)] for j in range(2)]
        self.arm_to(arm_ik.HOME, 'home')
        windows = []
        for v, (vx, vy) in enumerate(chosen):
            self.fb(0.05 + 0.45 * v, f'{room} bakis noktasi {v+1}/{len(chosen)}: ({vx:.1f}, {vy:.1f})')
            rx, ry, _ = self.map_pose()
            if math.hypot(vx - rx, vy - ry) > 0.25:
                self.go(vx, vy, math.atan2(vy - ry, vx - rx))
            # 8 adımda 45°'lik dönüşler (Nav2 Spin). Her adımdan sonra robot DURURKEN 1.5 s
            # gözlem: dönerken kare/TF zaman kayması konumları dağıtır.
            for s8 in range(8):
                self.check_cancel()
                self.nav.spin(math.pi / 4, cancel_check=self.cancelled)
                self.wait_sim(0.5)
                t_obs = self.now_sim()
                self.wait_sim(1.5)
                windows.append((t_obs, self.now_sim()))
        # Algılayıcı yük altında kareleri gecikmeli işleyebilir: son sonuçların da
        # gelmesini bekle, sonra zaman damgası pencerelere düşenleri topla.
        self.wait_sim(1.5)
        dets = [d for a, b in windows for d in self.recent_detections(a, b)]

        clusters = []   # [color, sx, sy, n]
        margin = 0.12
        for d in dets:
            px, py = d.position_map.x, d.position_map.y
            if not (x0 + margin < px < x1 - margin and y0 + margin < py < y1 - margin):
                continue    # Başka odadan (açık kapıdan) görülen
            if self.in_bin_zone(px, py):
                continue    # Kutunun içi ve etiketli ön yüzünün önü (hayalet şeritler)
            for c in clusters:
                if c[0] == d.color and math.hypot(c[1] / c[3] - px, c[2] / c[3] - py) < 0.18:
                    c[1] += px
                    c[2] += py
                    c[3] += 1
                    break
            else:
                clusters.append([d.color, px, py, 1])

        min_obs = self.get_parameter('min_observations').value
        from collections import Counter
        self.get_logger().info(
            f'Tarama: {len(dets)} ham algi {dict(Counter(d.color for d in dets))}; kumeler: '
            + ', '.join(f'{c}({sx/n:.2f},{sy/n:.2f})x{n}' for c, sx, sy, n in clusters))
        found = []
        for color, sx, sy, n in clusters:
            if n < min_obs:
                continue
            x, y = sx / n, sy / n
            # Zaten kayıtlıysa güncelle
            existing = [k for k, p in self.pieces.items()
                        if p['color'] == color and p['state'] == 'free'
                        and math.hypot(p['x'] - x, p['y'] - y) < 0.2]
            if existing:
                p = self.pieces[existing[0]]
                p['x'], p['y'], p['obs'] = x, y, p['obs'] + n
                found.append(existing[0])
                continue
            idx = sum(1 for k in self.pieces if k.startswith(f'{color}_{room}_'))
            name = f'{color}_{room}_{idx}'
            self.pieces[name] = {'color': color, 'room': room, 'x': x, 'y': y, 'obs': n, 'state': 'free'}
            found.append(name)
        summary = ', '.join(f'{n}@({self.pieces[n]["x"]:.2f},{self.pieces[n]["y"]:.2f})' for n in found)
        self.fb(1.0, f'{len(found)} parca bulundu: {summary}')
        return f'{len(found)} parca: {summary}'

    # ---------------------------------------------------------------- pick
    def map_to_odom(self, x, y):
        """map noktasını odom çerçevesine çevir (en güncel AMCL tahmini)."""
        t = self.tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
        tx, ty = t.transform.translation.x, t.transform.translation.y
        c, s_ = math.cos(quat_to_yaw(t.transform.rotation)), math.sin(quat_to_yaw(t.transform.rotation))
        return tx + c * x - s_ * y, ty + s_ * x + c * y

    def obstacles_odom(self, exclude=(), include_recent=True, recent_s=4.0, exclude_r=0.07, skip=None):
        """
        Taban hareketlerinde ÇARPILMAMASI gereken küpler (odom çerçevesinde):
          * dünya modelindeki yerdeki (free) küpler,
          * son 'recent_s' saniyede kameranın gördüğü (henüz kayıtlı olmayan) küpler.
        'exclude': hedef küp gibi yanaşılacak noktalar (odom).
        """
        pts = []
        try:
            for p in self.pieces.values():
                if p['state'] == 'free' and p is not skip:
                    pts.append(self.map_to_odom(p['x'], p['y']))
        except Exception:  # noqa: BLE001  (TF yoksa sadece canlı algılar)
            pass
        if include_recent:
            for d in self.recent_detections(self.now_sim() - recent_s):
                if self.in_bin_zone(d.position_map.x, d.position_map.y):
                    continue     # kutu ön yüzündeki hayalet şeritler
                pts.append((d.position_odom.x, d.position_odom.y))
        out = []
        for q in pts:
            if any(math.hypot(q[0] - e[0], q[1] - e[1]) < exclude_r for e in exclude):
                continue
            if any(math.hypot(q[0] - o[0], q[1] - o[1]) < 0.03 for o in out):
                continue     # aynı küpün tekrar eden ölçümleri
            out.append(q)
        return out

    def motion_guard(self, obst, lin=0.0, ang=0.0, margin=0.015):
        """drive()/rotate() için koruma: kısa vadede varılacak pozda gövde bir küpe değecek mi?"""
        def guard(remaining):
            rx, ry, ryaw = self.base.pose('odom')
            step_l = math.copysign(min(abs(remaining), 0.04), remaining) if lin else 0.0
            step_a = math.copysign(min(abs(remaining), 0.15), remaining) if ang else 0.0
            yaw = ryaw + step_a
            px, py = rx + step_l * math.cos(ryaw), ry + step_l * math.sin(ryaw)
            hit = pose_conflicts(obst, px, py, yaw, margin)
            if hit:
                bx, by = to_local(hit[0][0], hit[0][1], rx, ry, ryaw)
                self.get_logger().warn(f'Carpisma korumasi: kup govdeye cok yakin (base {bx:.2f},{by:.2f}); '
                                       'hareket durduruldu')
            return bool(hit)
        return guard

    def safe_drive(self, dist, exclude=(), include_recent=True, **kw):
        obst = self.obstacles_odom(exclude, include_recent)
        return self.base.drive(dist, guard=self.motion_guard(obst, lin=1.0), **kw)

    def safe_rotate(self, angle, exclude=(), include_recent=True, **kw):
        obst = self.obstacles_odom(exclude, include_recent)
        return self.base.rotate(angle, guard=self.motion_guard(obst, ang=1.0), **kw)

    def wait_track(self, n, seconds):
        """Takipçi en az n ölçüm toplayana kadar (sim zamanında) bekle."""
        t_end = self.now_sim() + seconds
        while self.now_sim() < t_end:
            if self.tracker.snapshot()['n_obs'] >= n:
                return True
            time.sleep(0.05)
        return self.tracker.snapshot()['n_obs'] >= n

    def target_base(self):
        """Takip edilen hedefin GÜNCEL robot pozuna göre konumu (base)."""
        ex, ey = self.tracker.snapshot()['est']
        return to_local(ex, ey, *self.base.pose('odom'))

    def servo_to_piece(self, goal_x, target=None, timeout=45.0):
        """
        GÖRSEL SERVO YANAŞMA: hedef küpü 20 Hz'de izleyerek (dur-ölç-sür yerine) sürekli
        hız komutlarıyla küpü (goal_x, 0) noktasına getir.
          * Hedef tahmini odom'da tutulur ve her karede güncellenir; küp kör bölgeye
            (<~0.24 m) girince son tahmin + taze odometri kullanılır.
          * Açı hatası büyükse önce yerinde dönülür (yanlamasına sürüşle küp itilmez).
          * Her adımda, ~0.3 s sonraki pozda gövdenin başka bir küpe değip değmeyeceği
            kontrol edilir; değecekse durulur ('blocked').
        Dönüş: ('ok', bx, by) | ('lost', ..) | ('blocked', ..) | ('timeout', ..)
        """
        v_max = self.get_parameter('approach_v_max').value
        est0 = self.tracker.snapshot()['est']
        obst = self.obstacles_odom(exclude=[est0], exclude_r=0.08, skip=target)
        t0 = time.monotonic()
        good = 0
        last_log = 0.0
        status = 'timeout'
        bx = by = 0.0
        try:
            while time.monotonic() - t0 < timeout:
                self.check_cancel()
                snap = self.tracker.snapshot()
                rx, ry, ryaw = self.base.pose('odom')
                bx, by = to_local(snap['est'][0], snap['est'][1], rx, ry, ryaw)
                e_th = math.atan2(by, bx)
                e_d = bx - goal_x
                age = (self.now_sim() - snap['last_seen']) if snap['last_seen'] is not None else 1e9
                # Görüş alanında olması gerektiği halde uzun süredir görünmüyor -> kayıp
                if age > 2.5 and bx > 0.30 and abs(e_th) < 0.40:
                    status = 'lost'
                    break
                if abs(e_d) < 0.004 and abs(by) < 0.004:
                    good += 1
                    if good >= 4:
                        status = 'ok'
                        break
                    self.base.send(0.0, 0.0)
                    time.sleep(0.05)
                    continue
                good = 0
                w = max(-0.6, min(0.6, 1.8 * e_th))
                if abs(by) > 0.003 and abs(w) < 0.04:
                    w = math.copysign(0.04, e_th)
                if abs(e_th) > 0.25:
                    v = 0.0                       # önce küpe dön
                else:
                    cap = v_max if e_d > 0.15 else (0.06 if e_d > 0.05 else 0.03)
                    v = max(-0.04, min(cap, 0.8 * e_d)) * (1.0 - abs(e_th) / 0.25)
                    if abs(e_d) >= 0.004 and abs(v) < 0.01:
                        v = math.copysign(0.01, e_d)
                # Çarpışma koruması (0.3 s ileri tahmin)
                hz = 0.3
                px = rx + v * hz * math.cos(ryaw)
                py = ry + v * hz * math.sin(ryaw)
                hit = pose_conflicts(obst, px, py, ryaw + w * hz, margin=0.015)
                if hit:
                    hb = to_local(hit[0][0], hit[0][1], rx, ry, ryaw)
                    self.get_logger().warn(f'Servo: yol uzerinde baska kup (base {hb[0]:.2f},{hb[1]:.2f})')
                    status = 'blocked'
                    break
                self.base.send(v, w)
                if time.monotonic() - last_log > 1.0:
                    last_log = time.monotonic()
                    self.get_logger().info(
                        f'   servo: kup base=({bx:.3f},{by:+.3f}) aci {math.degrees(e_th):+.1f}deg '
                        f'v={v:.3f} w={w:+.2f} olcum={snap["n_obs"]} yas={min(age, 99):.1f}s')
                time.sleep(0.05)
        finally:
            self.base.stop()
        time.sleep(0.3)
        bx, by = self.target_base()
        self.get_logger().info(f'Servo sonucu: {status} kup base=({bx:.3f},{by:+.3f}) '
                               f'olcum={self.tracker.snapshot()["n_obs"]} kayma={self.tracker.drift()*1000:.0f}mm')
        return status, bx, by

    def reacquire(self, piece, p):
        """
        Hedef beklenen yerde görünmüyor: büyük olasılıkla itildi ya da kör noktada.
        Biraz geri çekil (robotun dibindeki küp görüş alanına girer), sağa-sola bak ve
        aynı renkte, beklenen yere yakın (≤0.9 m), başka kayıtlı parçaya ait olmayan bir küp ara.
        """
        self.fb(0.32, f'{piece} beklenen yerde yok: yakin cevrede araniyor')
        guess = self.map_to_odom(p['x'], p['y'])
        self.safe_drive(-0.25, exclude=[guess])
        rx, ry, ryaw = self.base.pose('odom')
        bx, by = to_local(guess[0], guess[1], rx, ry, ryaw)
        self.safe_rotate(math.atan2(by, bx), exclude=[guess])
        others = [(q['x'], q['y']) for n, q in self.pieces.items() if n != piece and q['state'] == 'free']
        cands = []
        for ang in (0.0, 0.5, -1.0):
            if ang:
                self.safe_rotate(ang, exclude=[guess])
            t0 = self.now_sim()
            self.wait_sim(1.5)
            for d in self.recent_detections(t0):
                if d.color != p['color']:
                    continue
                mx, my = d.position_map.x, d.position_map.y
                if math.hypot(mx - p['x'], my - p['y']) > 0.9 or self.in_bin_zone(mx, my):
                    continue
                if any(math.hypot(mx - ox, my - oy) < 0.12 for ox, oy in others):
                    continue
                cands.append(d)
        if len(cands) < 3:
            return None
        # Beklenen yere en yakın küme
        cands.sort(key=lambda d: math.hypot(d.position_map.x - p['x'], d.position_map.y - p['y']))
        ref = cands[0].position_odom
        sel = [d for d in cands if math.hypot(d.position_odom.x - ref.x, d.position_odom.y - ref.y) < 0.05]
        if len(sel) < 3:
            return None
        ox = float(np.median([d.position_odom.x for d in sel]))
        oy = float(np.median([d.position_odom.y for d in sel]))
        mx = float(np.median([d.position_map.x for d in sel]))
        my = float(np.median([d.position_map.y for d in sel]))
        self.get_logger().warn(f'{piece} yeniden bulundu: ({p["x"]:.2f},{p["y"]:.2f}) -> ({mx:.2f},{my:.2f})')
        p['x'], p['y'] = mx, my
        return ox, oy

    def standoff_pose(self, px, py, dist, room, avoid_dirs=()):
        """
        Parçaya 'dist' mesafede, ona bakan bir yaklaşma pozu. Elenenler: duvara/kutuya yakın
        pozlar, başka küpe yakın pozlar ve robotun pozdan küpe düz yanaşırken gövdesinin
        süpüreceği koridorda başka küp bulunan yönler.
        """
        cx, cy = self.world['rooms'][room]['center']
        x0, x1, y0, y1 = self.world['rooms'][room]['bounds']
        base_dir = math.atan2(py - cy, px - cx)  # Oda merkezinden parçaya
        others = [(p['x'], p['y']) for p in self.pieces.values() if p['state'] == 'free'
                  and math.hypot(p['x'] - px, p['y'] - py) > 0.03]
        # Dünya modelinde olmayan ama kameranın son 20 s'de gördüğü küpler de engeldir
        for d in self.recent_detections(self.now_sim() - 20.0):
            mx, my = d.position_map.x, d.position_map.y
            if math.hypot(mx - px, my - py) < 0.12 or self.in_bin_zone(mx, my):
                continue
            if all(math.hypot(mx - ox, my - oy) > 0.05 for ox, oy in others):
                others.append((mx, my))
        fallback = None
        for k in [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6]:
            a = base_dir + k * math.radians(30)
            if any(abs(norm_angle(a - b)) < math.radians(20) for b in avoid_dirs):
                continue
            sx, sy = px - dist * math.cos(a), py - dist * math.sin(a)
            if not (x0 + 0.35 < sx < x1 - 0.35 and y0 + 0.35 < sy < y1 - 0.35):
                continue
            if self.near_bin(sx, sy, 0.45):
                continue
            if any(math.hypot(sx - ox, sy - oy) < 0.35 for ox, oy in others):
                continue
            # Koridor: küpten robota doğru yön = a + pi
            if corridor_blocked((px, py), a + math.pi, dist, others):
                fallback = fallback or (sx, sy, a)
                continue
            return sx, sy, a
        if fallback:
            return fallback
        return px - dist * math.cos(base_dir), py - dist * math.sin(base_dir), base_dir

    def approach_piece(self, piece, p):
        """
        Nav2 ile yaklaşma pozu + hedefi kilitleme + görsel servo. Yol bir küple tıkanırsa
        farklı bir yönden bir kez daha dener. Dönüş: (bx, by) kavrama noktası (base).
        """
        gx = self.get_parameter('grasp_x').value
        avoid = []
        for attempt in range(2):
            sx, sy, syaw = self.standoff_pose(p['x'], p['y'], 0.50, p['room'], avoid)
            self.fb(0.1 + 0.05 * attempt, f'{piece} icin yaklasma pozuna gidiliyor')
            self.arm_to(arm_ik.HOME, 'home')
            # Hedef robotun dibindeyse (kör bölge / önüne katılmış küp) Nav2 ile dönmeye
            # başlamadan önce düz geri çekil: aksi halde dönüş/kalkış küpü sürükler.
            tbx, tby = self.to_base(p['x'], p['y'])
            if 0.0 < tbx < 0.42 and abs(tby) < 0.25:
                self.get_logger().info(f'Hedef cok yakin (base {tbx:.2f},{tby:.2f}): once geri cekiliyor')
                self.safe_drive(-(0.45 - tbx), exclude=[self.map_to_odom(p['x'], p['y'])])
            self.go(sx, sy, syaw)

            # Hedefi kilitle: dünya modelindeki konum -> odom; takipçi en yakın aynı renkli küpü izler
            self.fb(0.3, 'hedef kup kilitleniyor')
            guess = self.map_to_odom(p['x'], p['y'])
            self.tracker.start(p['color'], guess, gate=0.20)
            if not self.wait_track(3, 2.0):
                bx, by = to_local(guess[0], guess[1], *self.base.pose('odom'))
                self.safe_rotate(math.atan2(by, bx), exclude=[guess])
                if not self.wait_track(3, 2.0):
                    found = self.reacquire(piece, p)
                    if found is None:
                        # Küp kayıtlı yerinde değil: dünya modelinden çıkar ki engel bulutu boşa
                        # kalmasın ve doğrulama taraması onu yeni yerinde yeni bir parça olarak bulsun
                        p['state'] = 'lost'
                        raise SkillError(f'{piece} kamerada gorunmuyor (cevrede de bulunamadi)')
                    self.tracker.start(p['color'], found, gate=0.10)
                    if not self.wait_track(3, 2.5):
                        p['state'] = 'lost'
                        raise SkillError(f'{piece} yeniden bulundu ama takip edilemedi')
            snap = self.tracker.snapshot()
            self.log_ground_truth(*self.target_base(), f'kilit n={snap["n_obs"]}')

            self.fb(0.45, 'gorsel servo ile yanasma')
            status, bx, by = self.servo_to_piece(gx, target=p)
            if status == 'ok' or (status == 'timeout' and abs(bx - gx) < 0.02 and abs(by) < 0.012):
                return bx, by
            if status == 'blocked':
                # Başka bir küp yolda: geri çekil, o yönden kaçınarak yeni bir yaklaşma pozu seç
                avoid.append(syaw)
                self.safe_drive(-0.20, exclude=[self.tracker.snapshot()['est']])
                continue
            if status == 'lost':
                p['state'] = 'lost'
                raise SkillError(f'{piece} yanasma sirasinda kayboldu')
            raise SkillError(f'yanasma tamamlanamadi ({status}, x={bx:.3f}, y={by:.3f})')
        raise SkillError(f'{piece}: yanasma yolu baska kuplerle tikali')

    def skill_pick(self, piece, *_):
        if piece not in self.pieces:
            raise SkillError(f'dunya modelinde yok: {piece}')
        p = self.pieces[piece]
        if self.held_model is not None:
            # Önceki bir hatadan kalan tutarsızlık (planlayıcı küpten vazgeçmiş ama robot hâlâ
            # tutuyor): sonraki işleri kilitlememesi için tutulan küpü bırak ve devam et.
            self.get_logger().warn(f'Gripper dolu ({self.held_model}): once birakiliyor')
            self.arm_to(arm_ik.HOME, 'home', required=False)
            self.gripper(GRIPPER_OPEN)
            self.attacher.detach(self.held_model)
            self.held_model = None
        if p['state'] == 'lost':
            p['state'] = 'free'      # planlayıcı tekrar denemek istiyor: çevrede aranacak
        grasp = self.get_parameter('gripper_grasp').value

        # 1-3) Nav2 + hedef kilitleme + görsel servo yanaşma
        try:
            bx, by = self.approach_piece(piece, p)
        finally:
            target_odom = self.tracker.snapshot()['est']
            self.tracker.stop()

        self.log_ground_truth(bx, by, 'kavrama')
        # 4) IK + MoveIt: ψ'yi olabildiğince dik seç; ön-kavrama noktası gripper EKSENİ
        #    boyunca 5 cm geride (eğik inişte parmaklar küpe yandan çarpmasın)
        z_grasp = self.world['piece_size'] / 2 + 0.003
        q_grasp, psi = self.arm_ik(bx, by, z_grasp, arm_ik.PSI_TOP_DOWN, 'kavrama')
        q1 = math.atan2(by, bx - arm_ik.X_J1)
        back = 0.05
        px = bx - back * math.cos(psi) * math.cos(q1)
        py = by - back * math.cos(psi) * math.sin(q1)
        pz = z_grasp - back * math.sin(psi)
        q_pre, _ = self.arm_ik(px, py, pz, [psi] + arm_ik.PSI_TOP_DOWN, 'on-kavrama')
        # Yüksek ara nokta: MoveIt küpü engel olarak bilmez; HOME'dan doğrudan ön-kavramaya
        # eklem uzayında giden yörünge gripper'ı küpün üzerinden süpürüp onu itebiliyordu
        # (teşhis: konum hatası ≤4 mm iken küp attach anında 4-12 cm uzaktaydı).
        q_above, _ = self.arm_ik(bx, by, 0.12, [psi] + arm_ik.PSI_TOP_DOWN, 'kup ustu')
        self.fb(0.6, f'kavrama: x={bx:.3f} y={by:.3f} psi={math.degrees(psi):.0f}deg')
        self.gripper(GRIPPER_OPEN)
        self.arm_to(q_above, 'kup ustu')
        self.arm_to(q_pre, 'on-kavrama', vel=0.3)
        self.arm_to(q_grasp, 'kavrama', vel=0.3)

        # 5) Sabitleme (attach) -> SONRA parmakları kapat.
        #    Neden bu sıra? Gripper eklemleri pozisyon arayüzünde (konum doğrudan atanır) çalışır; iki parmak
        #    küpe biraz fazla girerse 30 g'lık küpe büyük itki verir ve onu 10 cm'den fazla uzağa itiyordu
        #    (teşhis: konum hatası 1 mm iken küp attach anında 12 cm uzaktaydı). Attach yine
        #    küpün kıskacın içinde olduğunu MESAFEYLE doğrular; ardından küpün çarpışması kapalı
        #    olduğu için kapanan parmaklar onu itmez.
        ok, model, dist = self.attacher.attach(max_distance=0.035)
        if ok:
            self.gripper(grasp)
            time.sleep(0.3)
        if not ok:
            self.gripper(GRIPPER_OPEN)
            self.arm_to(q_above, 'geri cekilme')
            self.arm_to(arm_ik.HOME, 'home')
            self.safe_drive(-0.15, exclude=[target_odom])
            raise SkillError(f'kavrama basarisiz: {model}')
        self.held_model = model
        p['state'] = 'held'
        self.fb(0.85, f'tutuldu ({model}, d={dist*100:.1f} cm), kaldiriliyor')
        self.arm_to(q_pre, 'kaldirma')
        self.arm_to(arm_ik.HOME, 'tasima pozu')
        self.safe_drive(-0.15, exclude=[target_odom])
        # Alınan küp costmap'te engel olarak işaretli kalmasın
        self.nav.clear_costmaps()
        return f'{piece} tutuldu'

    # ------------------------------------------------------------ test/teşhis
    def skill_debug_reset(self, *_):
        """Alt simülasyon testleri için: dünya modelini ve tutulan küpü sıfırla."""
        self.tracker.stop()
        if self.held_model is not None:
            self.gripper(GRIPPER_OPEN)
            self.attacher.detach(self.held_model)
            self.held_model = None
        self.pieces.clear()
        self.scan_count.clear()
        self.arm_to(arm_ik.HOME, 'home', required=False)
        self.nav.clear_costmaps()
        return 'sifirlandi'

    def skill_debug_add_piece(self, name, color, room, x, y, *_):
        """Alt simülasyon testleri için: taramayı atlayıp dünya modeline parça ekle."""
        self.pieces[name] = {'color': color, 'room': room, 'x': float(x), 'y': float(y),
                             'obs': 99, 'state': 'free'}
        return f'{name} eklendi'

    # ---------------------------------------------------------------- place
    def skill_place(self, piece, bin_name, *_):
        """
        Kutuya bırakma (AprilTag referanslı):
          1. Nav2 ile kutunun ~0.8 m önüne (kutuya bakarak)
          2. Kutunun ön yüzündeki etikete iki aşamalı hizalan (0.50 m, sonra 0.30 m)
          3. Kutu merkezi = etiket + 0.11 m (etiketin arkası, -n yönünde); merkez 0.24 m önde
             olacak şekilde odometriyle yanaş (kısa mesafe: ±2 mm)
          4. Kolu kutu merkezinin üstüne getir, bırak
        """
        if self.held_model is None:
            # İdempotent: önceki denemede bırakılmış ama sonrasında hata verilmişse
            if self.pieces.get(piece, {}).get('state') == 'placed':
                return f'{piece} zaten {bin_name} icinde'
            raise SkillError('gripper bos')
        b = self.world['bins'][bin_name]
        bx_m, by_m = b['position']
        yaw = b['approach_yaw']
        tx = self.get_parameter('place_target_x').value
        half = b['size'] / 2

        self.fb(0.1, f'{bin_name} kutusuna gidiliyor')
        self.go(bx_m - 0.80 * math.cos(yaw), by_m - 0.80 * math.sin(yaw), yaw)
        self.fb(0.3, f'AprilTag {b["tag_id"]} ile hizalanma')
        self.face_tag_at(b['tag_id'], 0.45)
        x, y, _, _ = self.face_tag_at(b['tag_id'], 0.30)
        # Kutu merkezi = etiket + yarım kutu (etiketin arkasına, kutunun içine doğru).
        # Bu yön için haritadaki BİLİNEN kutu yönelimi kullanılır (robot yönelimi AMCL'den,
        # ±2° -> 0.11 m'de ~4 mm); PnP normali küçük etikette daha gürültülüdür.
        _, _, ryaw = self.map_pose()
        inward = norm_angle(yaw - ryaw)          # kutunun içine doğru yön, base çerçevesinde
        cx, cy = x + half * math.cos(inward), y + half * math.sin(inward)
        self.get_logger().info(f'Kutu merkezi (etiketten) base: ({cx:.3f}, {cy:.3f})')
        self.fb(0.5, f'kutuya yanasma {cx - tx:.3f} m')
        self.base.drive(cx - tx)
        bx, by = tx, cy

        z_drop = b['height'] + 0.06
        q_drop, psi = self.arm_ik(bx, by, z_drop, arm_ik.PSI_PLACE, 'birakma')
        self.fb(0.6, f'birakma: x={bx:.3f} y={by:.3f} psi={math.degrees(psi):.0f}deg')
        if not self.arm_to(q_drop, 'birakma', required=False):
            # Kol kutu kenarına takılmış olabilir: yine de kutunun üstünde/yanında, bırak.
            self.get_logger().warn('Birakma pozuna tam ulasilamadi, bulunulan yerde birakiliyor')
        self.gripper(GRIPPER_OPEN)
        self.attacher.detach(self.held_model)
        time.sleep(0.8)
        self.held_model = None
        self.pieces.get(piece, {})['state'] = 'placed'
        if not self.arm_to(arm_ik.HOME, 'home', required=False):
            self.safe_drive(-0.10, include_recent=False)
            self.arm_to(arm_ik.HOME, 'home (geri cekildikten sonra)', required=False)
        self.safe_drive(-0.25, include_recent=False)
        return f'{piece} -> {bin_name}'

    # ------------------------------------------------------------ open_door
    def front_distance(self):
        """Lidarın tam önündeki (±3°) en yakın engel mesafesi (base_scan'dan)."""
        s = self.scan
        if s is None:
            return None
        vals = []
        for i, r in enumerate(s.ranges):
            a = norm_angle(s.angle_min + i * s.angle_increment)
            if abs(a) < math.radians(3) and s.range_min < r < s.range_max:
                vals.append(r)
        return float(np.median(vals)) if vals else None

    def skill_open_door(self, door, *_):
        d = self.world['doors'][door]
        if self.doors.get(door) and self.doors[door].open:
            return f'{door} zaten acik'
        px, py, pyaw = d['pre_button_pose']
        bxm, bym, bz = d['button_pos']
        tx = self.get_parameter('press_target_x').value

        self.fb(0.1, f'{door} butonuna gidiliyor')
        self.arm_to(arm_ik.HOME, 'home')
        self.go(px, py, pyaw)
        # Duvara tam dik dur: önce AMCL yönelimiyle kaba, sonra lidar ile hassas
        _, _, yaw = self.map_pose()
        self.base.rotate(norm_angle(pyaw - yaw))
        self.align_to_front_wall()
        # Yanal hata düzeltmesi: Nav2 ~10 cm toleransla durur, kol ise ±2 cm ister.
        # 90° dön -> yanal farkı odometriyle git -> geri dön (3 noktalı manevra)
        _, by = self.to_base(bxm, bym)
        if abs(by) > 0.02:
            self.fb(0.3, f'yanal duzeltme {by*100:.1f} cm')
            side = math.copysign(math.pi / 2, by)
            self.base.rotate(side)
            self.base.drive(abs(by))
            _, _, yaw = self.map_pose()
            self.base.rotate(norm_angle(pyaw - yaw))
            self.align_to_front_wall()

        # Lidar ile duvara mesafe -> butonun base x konumu (kapalı döngü, 2 tur)
        # base_scan, base_footprint'e göre x=-0.024; buton merkezi duvardan 0.015 önde
        def button_x_now():
            fd = self.front_distance()
            if fd is None:
                return self.to_base(bxm, bym)[0]
            return fd - 0.024 - 0.015
        for k in range(2):
            button_x = button_x_now()
            err = button_x - tx
            self.get_logger().info(f'Buton x={button_x:.3f} (hedef {tx:.3f})')
            if abs(err) < 0.01:
                break
            self.fb(0.4 + 0.05 * k, f'butona yanasma {err:.3f} m')
            self.base.drive(err)
        self.align_to_front_wall()
        button_x = button_x_now()
        _, by = self.to_base(bxm, bym)
        if abs(by) > 0.06:
            raise SkillError(f'buton yanal hatasi cok buyuk ({by:.3f} m)')

        # Basma: yatay itme, önce 6 cm geriden, sonra buton merkezinin 0.5 cm ötesine
        for k, dy in enumerate([0.0, 0.03, -0.03]):
            y = by + dy
            q_pre, psi = self.arm_ik(button_x - 0.06, y, bz, arm_ik.PSI_PRESS, 'on-basma')
            q_press, _ = self.arm_ik(button_x + 0.005, y, bz, [psi] + arm_ik.PSI_PRESS, 'basma')
            self.fb(0.6 + 0.1 * k, f'butona basiliyor (x={button_x:.3f}, y={y:.3f})')
            self.gripper(GRIPPER_CLOSED)
            self.arm_to(q_pre, 'on-basma')
            # Kol butona/duvara FİZİKSEL olarak temas eder (tork kontrolü): hedefe tam
            # ulaşamaması beklenir. Başarı ölçütü MoveIt sonucu değil, kapının durumudur.
            self.check_cancel()
            if not self.moveit.arm(q_press, 0.2):
                self.get_logger().info('Basma hareketi temas nedeniyle tamamlanamadi (beklenen)')
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0:
                st = self.doors.get(door)
                if st and (st.button_pressed or st.opening > 0.0):
                    break
                time.sleep(0.1)
            self.arm_to(q_pre, 'geri cekilme')
            st = self.doors.get(door)
            if st and (st.button_pressed or st.opening > 0.0):
                break
        else:
            self.arm_to(arm_ik.HOME, 'home')
            raise SkillError(f'{door} butonuna basilamadi')

        self.arm_to(arm_ik.HOME, 'home')
        self.base.drive(-0.30)
        self.fb(0.9, 'kapinin acilmasi bekleniyor')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0:
            st = self.doors.get(door)
            if st and st.open:
                self.nav.clear_costmaps()
                return f'{door} acildi'
            time.sleep(0.2)
        raise SkillError(f'{door} acilmadi')

    # ---------------------------------------------------------------- charge
    def scan_points(self, a_min, a_max, max_r=1.5):
        """Lidar noktaları (base_scan çerçevesinde) - açı aralığı [a_min, a_max] (rad)."""
        sc = self.scan
        pts = []
        if sc is None:
            return pts
        for i, rr in enumerate(sc.ranges):
            a = norm_angle(sc.angle_min + i * sc.angle_increment)
            if a_min <= a <= a_max and sc.range_min < rr < min(max_r, sc.range_max):
                pts.append((rr * math.cos(a), rr * math.sin(a)))
        return pts

    def right_wall(self):
        """
        Sağdaki duvara doğru uydur (y = m·x + c, en küçük kareler).
        Dönüş: (robotun duvara paralel olması için dönmesi gereken açı, duvara dik mesafe)
        """
        pts = self.scan_points(math.radians(-110), math.radians(-70))
        if len(pts) < 10:
            return None
        xs, ys = np.array(pts).T
        m, c = np.polyfit(xs, ys, 1)
        return math.atan(m), abs(c) / math.sqrt(1 + m * m)

    def front_wall_yaw_error(self):
        """
        Öndeki duvara (±25°) x = m·y + c doğrusunu uydur. Robot duvara tam dik ise m = 0;
        aksi halde atan(m) robotun duvara göre yönelim hatasıdır (AMCL'den bağımsız).
        """
        pts = self.scan_points(math.radians(-25), math.radians(25), 1.2)
        if len(pts) < 10:
            return None
        xs, ys = np.array(pts).T
        m, _ = np.polyfit(ys, xs, 1)
        return math.atan(m)

    def align_to_front_wall(self, tries=2):
        for _ in range(tries):
            e = self.front_wall_yaw_error()
            if e is None:
                return
            self.get_logger().info(f'Duvara dik hizalama: yonelim hatasi {math.degrees(e):.1f} deg')
            if abs(e) < math.radians(0.7):
                return
            self.base.rotate(-e)

    def rear_distance(self):
        pts = self.scan_points(math.pi - math.radians(3), math.pi, 2.0) + \
            self.scan_points(-math.pi, -math.pi + math.radians(3), 2.0)
        return float(np.median([math.hypot(x, y) for x, y in pts])) if pts else None

    # ------------------------------------------------------------ AprilTag
    def measure_tag(self, tag_id, window=1.0):
        """
        Son 'window' saniyede görülen etiketin base_footprint'teki pozu (medyan).
        Dönüş: (x, y, nx, ny) — etiket merkezi ve yüzey normali (robota doğru, birim, 2B).

        Poz, apriltag_ros'un TF'i yerine etiketin KÖŞE piksellerinden burada solvePnP ile
        hesaplanır: etiket TF'leri görüntü zaman damgasında tampona geç ulaşıyordu
        ("extrapolation into the future"); köşeler mesajın içinde gelir, kamera->base
        dönüşümü ise statiktir -> hiçbir zamanlama bağımlılığı yok.
        """
        t0 = self.now_sim()
        self.wait_sim(window)
        t1 = self.now_sim()
        self.wait_sim(0.2)
        if self.cam_K is None:
            return None
        with self.tag_lock:
            dets = [c for t, i, c in self.tag_seen if i == tag_id and t0 <= t <= t1]
        if len(dets) < 3:
            return None
        h = self.tag_size(tag_id) / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        R_bc, t_bc = self.cam_to_base()
        xs, ys, ns = [], [], []
        for c in dets:
            best = None
            # Köşe sırası (saat yönü / tersi) belirsizliğine karşı iki sırayı da dene
            for img in (c, c[::-1].copy()):
                ok, rvec, tvec = cv2.solvePnP(obj, img, self.cam_K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                proj, _ = cv2.projectPoints(obj, rvec, tvec, self.cam_K, None)
                err = float(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1).mean())
                if best is None or err < best[0]:
                    best = (err, rvec, tvec)
            if best is None or best[0] > 2.0:
                continue
            R_ct, _ = cv2.Rodrigues(best[1])
            p = R_bc @ best[2].reshape(3) + t_bc
            n = R_bc @ R_ct[:, 2]
            xs.append(p[0])
            ys.append(p[1])
            ns.append(math.atan2(n[1], n[0]))
        if len(xs) < 3:
            return None
        x, y = float(np.median(xs)), float(np.median(ys))
        a = math.atan2(np.median(np.sin(ns)), np.median(np.cos(ns)))
        nx, ny = math.cos(a), math.sin(a)
        if nx * (-x) + ny * (-y) < 0:      # normal robota baksın
            nx, ny = -nx, -ny
        return x, y, nx, ny

    def find_tag(self, tag_id):
        """Etiket görünmüyorsa yerinde küçük açılarla dönerek ara."""
        m = self.measure_tag(tag_id)
        for ang in (0.35, -0.70, 1.05, -1.40):
            if m is not None:
                return m
            self.base.rotate(ang)
            m = self.measure_tag(tag_id)
        return m

    def face_tag_at(self, tag_id, dist, iters=4):
        """
        Etikete DÖN ve etiketten 'dist' metre uzakta dur — sadece etiketin KONUMUNU kullanır
        (PnP'nin konum kestirimi ~1 mm doğrudur; yönelim kestirimi küçük etiketlerde düzlemsel
        belirsizlik yüzünden ±4° zıplayabilir). Robot etikete tam dik olmak zorunda değildir.
        """
        m = None
        for k in range(iters):
            self.check_cancel()
            m = self.find_tag(tag_id)
            if m is None:
                raise SkillError(f'AprilTag {tag_id} gorulemedi')
            x, y = m[0], m[1]
            bearing = math.atan2(y, x)
            derr = math.hypot(x, y) - dist
            self.get_logger().info(f'[Tag {tag_id}] tur {k+1}: etiket=({x:.3f},{y:.3f}) '
                                   f'aci {math.degrees(bearing):.1f} deg, mesafe hatasi {derr*1000:.0f} mm')
            if abs(bearing) < 0.021 and abs(derr) < 0.008:   # 1.2°: kutu için fazlasıyla yeterli
                return m
            if abs(bearing) >= 0.021:
                self.base.rotate(bearing)
            if abs(derr) >= 0.008:
                self.base.drive(derr, max_v=0.06)
        return self.measure_tag(tag_id) or m

    def align_to_tag(self, tag_id, dist, tol_pos=0.025, tol_yaw=0.06, iters=2, world_normal=None):
        """
        Robotu, etiketin yüzey normali üzerinde etiketten 'dist' metre uzağa ve etikete
        TAM DİK bakacak şekilde getir.
          hedef nokta  T = etiket + dist * n      (n: robota bakan normal)
          hedef yönelim  = -n yönü
        Her turda: T'ye dön -> T'ye git -> etikete dön -> yeniden ölç.
        """
        for k in range(iters):
            self.check_cancel()
            m = self.find_tag(tag_id)
            if m is not None:
                m2 = self.measure_tag(tag_id, window=1.5)   # yönelim için daha uzun ortalama
                m = m2 or m
            if m is None:
                raise SkillError(f'AprilTag {tag_id} gorulemedi')
            x, y, nx, ny = m
            if world_normal is not None:
                # Normal füzyonu: PnP normali (küçük etikette ±4-6° gürültülü) ile haritadaki
                # bilinen yüzey yönünün (AMCL yönelimi üzerinden, ±2°) dairesel ortalaması
                _, _, ryaw = self.map_pose()
                a_map = norm_angle(world_normal - ryaw)
                a_pnp = math.atan2(ny, nx)
                a = math.atan2(math.sin(a_map) + math.sin(a_pnp), math.cos(a_map) + math.cos(a_pnp))
                nx, ny = math.cos(a), math.sin(a)
            tx, ty = x + dist * nx, y + dist * ny
            heading = math.atan2(-ny, -nx)
            pos_err = math.hypot(tx, ty)
            self.get_logger().info(
                f'[Tag {tag_id}] tur {k+1}: etiket=({x:.3f},{y:.3f}) konum hatasi {pos_err*1000:.0f} mm, '
                f'yonelim hatasi {math.degrees(heading):.1f} deg')
            if pos_err < tol_pos and abs(heading) < tol_yaw:
                return x, y, nx, ny
            if pos_err >= tol_pos:
                ang = math.atan2(ty, tx)
                d = pos_err
                if abs(ang) > math.pi / 2:           # hedef arkada: geri geri git
                    ang = norm_angle(ang + math.pi)
                    d = -pos_err
                self.base.rotate(ang)
                self.base.drive(d, max_v=0.06)
                self.base.rotate(norm_angle(heading - ang))
            else:
                self.base.rotate(heading)
        m = self.measure_tag(tag_id)
        if m is None:
            raise SkillError(f'AprilTag {tag_id} hizalama sonrasi gorulemedi')
        return m

    # ---------------------------------------------------------------- charge
    def skill_charge(self, dock_name, *_):
        """
        Dock'a yanaşma (AprilTag referanslı):
          1. Nav2 ile ön-poza git (istasyona BAKARAK)
          2. İstasyon etiketine iki aşamalı hizalan (0.60 m, sonra 0.45 m): yanal ±6 mm, yönelim ±0.7°
          3. 180° dön, arka lidar ile istasyon yüzüne mesafeyi kapalı döngüde ayarlayarak geri geri gir
          4. Batarya dolana kadar bekle, çık
        """
        dk = self.world['dock']
        sx, sy, syaw = dk['staging']
        self.fb(0.05, 'dock on-pozuna gidiliyor')
        self.arm_to(arm_ik.HOME, 'home')
        self.go(sx, sy, syaw)
        self.fb(0.15, 'AprilTag ile hizalanma')
        # Etiketin normali üzerine gel (dik dur). İstasyon yüzü odaya, yani -y'ye bakar.
        self.align_to_tag(dk['tag_id'], 0.60, world_normal=-math.pi / 2)
        self.face_tag_at(dk['tag_id'], 0.45)        # etikete tam dön, mesafeyi ayarla
        self.base.rotate(math.pi)

        self.fb(0.25, 'geri geri yanasma (lidar)')
        for k in range(3):
            rd = self.rear_distance()
            if rd is None:
                raise SkillError('arka lidar mesafesi yok')
            err = rd - dk['rear_dist']
            self.get_logger().info(f'Arka mesafe {rd:.3f} m (hedef {dk["rear_dist"]:.3f})')
            if abs(err) < 0.008:
                break
            self.base.drive(-err, max_v=0.06)

        self.fb(0.35, 'sarj ediliyor')
        t0 = time.monotonic()
        last_log = 0
        while True:
            self.check_cancel()
            b = self.battery
            if b is not None:
                pct = b.percentage * 100
                charging = b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                if time.monotonic() - last_log > 5:
                    self.fb(0.35 + 0.6 * pct / 100, f'batarya {pct:.0f}% {"(sarj oluyor)" if charging else ""}')
                    last_log = time.monotonic()
                if pct >= 99.0:
                    break
                if not charging and time.monotonic() - t0 > 6.0:
                    raise SkillError('dock pozunda sarj baslamadi')
            if time.monotonic() - t0 > 240:
                raise SkillError('sarj zaman asimi')
            time.sleep(0.5)
        self.fb(0.97, 'dock tan cikiliyor')
        self.base.drive(0.45)
        return 'batarya dolu'


def main():
    rclpy.init()
    node = SkillServer()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    # BasicNavigator kendi düğümüdür; kendi metotları içinde spin eder.
    try:
        node.moveit.wait_ready(120.0)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
