"""
Düşük seviye robot arayüzleri (beceri sunucusunun "kas ve sinir sistemi").

  Nav2Api      -> nav2_simple_commander.BasicNavigator ile NavigateToPose ve Spin
  MoveItApi    -> MoveIt 'move_group' düğümünün /move_action (MoveGroup) action'ı
  BaseMotion   -> /cmd_vel ile hassas kısa mesafe hareketleri (yanaşma, dock)
  Attacher     -> /mrm/attach ve /mrm/detach (Gazebo LinkAttacher plugin)

Hepsi senkron (bloklayan) metotlar sunar; çağıran thread beklerken ROS
callback'leri MultiThreadedExecutor'ın diğer thread'lerinde işlenir.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from moveit_msgs.action import MoveGroup
from nav_msgs.msg import Odometry
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from mrm_interfaces.srv import Attach
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.parameter import Parameter


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def wait_future(future, timeout, cancel_check=None):
    """Bir rclpy Future'ı (başka thread'de spin edilirken) bekle."""
    t0 = time.monotonic()
    while not future.done():
        if cancel_check and cancel_check():
            return False
        if timeout is not None and time.monotonic() - t0 > timeout:
            return False
        time.sleep(0.02)
    return True


class Nav2Api:
    """nav2_simple_commander üzerinden Nav2 kullanımı."""

    def __init__(self, logger):
        self.log = logger
        self.nav = BasicNavigator('mrm_skill_navigator')
        self.nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rclpy.time.Time().to_msg()  # 0 = "en güncel TF"
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        q = yaw_to_quat(yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        return p

    def go_to(self, x, y, yaw, timeout=180.0, cancel_check=None, feedback=None):
        """NavigateToPose action'ı: global planlayıcı + DWB yerel kontrolcü."""
        self.log.info(f'[Nav2] hedef ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.nav.goToPose(self._pose(x, y, yaw))
        t0 = time.monotonic()
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb and feedback:
                feedback(f'Nav2: kalan mesafe {fb.distance_remaining:.2f} m')
            if (cancel_check and cancel_check()) or time.monotonic() - t0 > timeout:
                self.nav.cancelTask()
                return False
            time.sleep(0.1)
        res = self.nav.getResult()
        if res != TaskResult.SUCCEEDED:
            self.log.warn(f'[Nav2] başarısız: {res}')
        return res == TaskResult.SUCCEEDED

    def spin(self, angle, timeout=60.0, cancel_check=None):
        """Nav2 behavior_server 'Spin' davranışı (yerel costmap ile çarpışma kontrollü)."""
        self.nav.spin(spin_dist=angle, time_allowance=int(timeout))
        t0 = time.monotonic()
        while not self.nav.isTaskComplete():
            if (cancel_check and cancel_check()) or time.monotonic() - t0 > timeout + 5:
                self.nav.cancelTask()
                return False
            time.sleep(0.1)
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def clear_costmaps(self):
        try:
            self.nav.clearAllCostmaps()
        except Exception:  # noqa: BLE001
            pass

    def shutdown(self):
        self.nav.destroy_node()


class MoveItApi:
    """
    MoveIt move_group'a MoveGroup action'ı ile EKLEM UZAYI hedefi gönderir.
    move_group: OMPL ile çarpışmasız yörünge planlar, zaman parametrelendirir
    ve arm_controller'a (FollowJointTrajectory) uygulatır.
    """

    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self, node, callback_group):
        self.node = node
        self.client = ActionClient(node, MoveGroup, '/move_action', callback_group=callback_group)

    def wait_ready(self, timeout=60.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def _goal(self, group, joints, values, vel):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = vel
        req.workspace_parameters.header.frame_id = 'base_footprint'
        c = Constraints()
        for n, v in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def move_joints(self, group, joints, values, vel=0.5, timeout=30.0):
        goal = self._goal(group, joints, values, vel)
        fut = self.client.send_goal_async(goal)
        if not wait_future(fut, 10.0) or not fut.result().accepted:
            self.node.get_logger().error('[MoveIt] hedef kabul edilmedi')
            return False
        res_fut = fut.result().get_result_async()
        if not wait_future(res_fut, timeout):
            fut.result().cancel_goal_async()
            self.node.get_logger().error('[MoveIt] zaman aşımı')
            return False
        code = res_fut.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(f'[MoveIt] hata kodu {code}')
        return code == MoveItErrorCodes.SUCCESS

    def arm(self, q, vel=0.5):
        return self.move_joints('arm', self.ARM_JOINTS, q, vel)

    def gripper(self, position):
        """gripper_left_joint: 0.019 tam açık, -0.010 tam kapalı (sağ parmak mimic)."""
        return self.move_joints('gripper', ['gripper_left_joint'], [position], vel=1.0, timeout=10.0)


class BaseMotion:
    """
    Nav2'nin yapamadığı/yapmasının istenmediği santimetre hassasiyetli hareketler.
    (Nav2 costmap'teki şişirme bölgesi nedeniyle duvara/kutuya 20 cm yaklaşamaz.)

    Poz kaynağı: doğrudan /odom mesajı (tekerlek odometrisi), AYRI bir düğümde ve
    AYRI bir thread'de dinlenir. Neden? Beceri sunucusu yük altındayken TF mesajları
    gecikmeli işleniyordu; drive() başlangıçta ESKİ bir pozu alıp, taze pozlar
    gelince önceki hareketin kalanını yeni hareket sanarak erken duruyordu
    (7 cm komut -> 4 mm hareket ölçüldü).
    """

    def __init__(self, node, tf_buffer, extra_executor):
        self.node = node
        self.tf = tf_buffer
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_node = rclpy.create_node(
            'skill_odom_listener',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom_node.create_subscription(Odometry, '/odom', self._on_odom, 50)
        extra_executor.add_node(self.odom_node)

    def _on_odom(self, msg):
        self.odom = msg.pose.pose

    def pose(self, frame='odom'):
        """(x, y, yaw) - 'odom': taze odometri; 'map': en güncel map->odom (AMCL) ∘ odometri."""
        t0 = time.monotonic()
        while self.odom is None and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        if self.odom is None:
            raise RuntimeError('/odom gelmiyor')
        p = self.odom
        x, y, yaw = p.position.x, p.position.y, quat_to_yaw(p.orientation)
        if frame == 'odom':
            return x, y, yaw
        t = self.tf.lookup_transform(frame, 'odom', rclpy.time.Time())
        tx, ty = t.transform.translation.x, t.transform.translation.y
        tyaw = quat_to_yaw(t.transform.rotation)
        c, s_ = math.cos(tyaw), math.sin(tyaw)
        return tx + c * x - s_ * y, ty + s_ * x + c * y, norm_angle(tyaw + yaw)

    def stop(self):
        self.pub.publish(Twist())

    def send(self, v, w):
        """Doğrudan hız komutu (görsel servo döngüleri için)."""
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)

    def rotate(self, angle, max_w=0.5, tol=0.015, timeout=20.0, guard=None):
        """
        Yerinde 'angle' radyan dön (P kontrol).
        guard(yaw_kalan) True dönerse dönüş yarıda kesilir (çarpışma koruması) -> False.
        """
        _, _, y0 = self.pose()
        target = norm_angle(y0 + angle)
        t0 = time.monotonic()
        ok = True
        while time.monotonic() - t0 < timeout:
            err = norm_angle(target - self.pose()[2])
            if abs(err) < tol:
                break
            if guard is not None and guard(err):
                ok = False
                break
            cmd = Twist()
            cmd.angular.z = max(-max_w, min(max_w, 2.0 * err))
            if abs(cmd.angular.z) < 0.08:
                cmd.angular.z = math.copysign(0.08, err)
            self.pub.publish(cmd)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)
        return ok

    def drive(self, dist, max_v=0.08, tol=0.004, timeout=30.0, guard=None):
        """
        Düz ileri (+) / geri (-) 'dist' metre git, başlangıç yönelimini koru.
        guard(kalan_mesafe) True dönerse sürüş yarıda kesilir (çarpışma koruması) -> False.
        """
        x0, y0, yaw0 = self.pose()
        t0 = time.monotonic()
        ok = True
        while time.monotonic() - t0 < timeout:
            x, y, yaw = self.pose()
            done = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
            err = dist - done
            if abs(err) < tol:
                break
            if guard is not None and guard(err):
                ok = False
                break
            cmd = Twist()
            v = max(-max_v, min(max_v, 1.5 * err))
            if abs(v) < 0.015:
                v = math.copysign(0.015, err)
            cmd.linear.x = v
            cmd.angular.z = 2.0 * norm_angle(yaw0 - yaw)
            self.pub.publish(cmd)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)
        return ok


class Attacher:
    def __init__(self, node, callback_group):
        self.node = node
        self.att = node.create_client(Attach, '/mrm/attach', callback_group=callback_group)
        self.det = node.create_client(Attach, '/mrm/detach', callback_group=callback_group)

    def _call(self, cli, piece, max_distance):
        req = Attach.Request()
        req.model_name_1, req.link_name_1 = 'mrm_robot', 'link5'
        req.model_name_2, req.link_name_2 = piece, 'link'
        # Parmak uçlarının ortası: link5 çerçevesinde 0.12 m ileride (uç efektör 0.126)
        req.link1_offset.x = 0.12
        req.max_distance = max_distance
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, 'service not available', -1.0
        fut = cli.call_async(req)
        if not wait_future(fut, 8.0):
            return False, 'timeout', -1.0
        r = fut.result()
        return r.success, r.message, r.distance

    def attach(self, max_distance=0.04):
        """Parmaklar arasındaki en yakın küpü bağla. Dönüş: (başarı, model_adı|hata, mesafe)."""
        return self._call(self.att, '', max_distance)

    def detach(self, piece):
        return self._call(self.det, piece, 0.0)
