#!/usr/bin/env python3
"""
Temsili batarya simülatörü.

Gerçek bir batarya modeli değildir; görev planlamasını tetiklemek için
basit ve öngörülebilir bir model kullanır:

    şarj(t+dt) = şarj(t) - drain_idle * dt - drain_per_meter * katedilen_mesafe

Robot şarj istasyonunun (dock) pozunda (charge_radius içinde, yönelimi doğru
ve duruyor) iken şarj `charge_rate` %/s hızla artar.

Yayınlar:
  /battery_state        sensor_msgs/BatteryState (percentage 0..1)
  /mrm/battery_marker   visualization_msgs/Marker (RViz'de robotun üstünde yazı)

Test için şarjı elle değiştirme:
  ros2 param set /battery_sim force_level 15.0
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BatterySim(Node):

    def __init__(self):
        super().__init__('battery_sim')
        default_cfg = os.path.join(get_package_share_directory('mrm_simulation'), 'config', 'world.yaml')
        self.declare_parameter('world_config', default_cfg)
        self.declare_parameter('initial_level', 100.0)     # %
        self.declare_parameter('drain_idle', 0.04)         # %/s (elektronik, sensörler)
        self.declare_parameter('drain_per_meter', 0.8)     # %/m (motorlar)
        self.declare_parameter('charge_rate', 2.5)         # %/s
        self.declare_parameter('force_level', -1.0)        # >=0 ise şarjı bu değere ayarla

        with open(self.get_parameter('world_config').value) as f:
            self.dock = yaml.safe_load(f)['dock']
        self.level = float(self.get_parameter('initial_level').value)
        self.charging = False
        self.last_odom = None
        self.speed = 0.0
        self.distance = 0.0

        self.true_pose = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.marker_pub = self.create_publisher(Marker, '/mrm/battery_marker', 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.last_t = self.get_clock().now()
        self.create_timer(0.5, self.tick)

    def on_params(self, params):
        for p in params:
            if p.name == 'force_level' and p.value >= 0.0:
                self.level = min(100.0, float(p.value))
                self.get_logger().warn(f'Batarya elle {self.level:.1f}% olarak ayarlandi')
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_odom is not None:
            self.distance += math.hypot(p.x - self.last_odom[0], p.y - self.last_odom[1])
        self.last_odom = (p.x, p.y)
        self.speed = abs(msg.twist.twist.linear.x) + 0.1 * abs(msg.twist.twist.angular.z)

    def on_models(self, msg):
        if 'mrm_robot' in msg.name:
            self.true_pose = msg.pose[msg.name.index('mrm_robot')]

    def at_dock(self):
        """
        Şarj kontakları değiyor mu? Bu FİZİKSEL bir olgudur: bu düğüm bir donanım
        simülatörü olduğu için robotun tahmini (AMCL) pozuna değil, Gazebo'daki
        GERÇEK pozuna bakar.
        """
        p = self.true_pose
        if p is None:
            return False
        dx = p.position.x - self.dock['pose'][0]
        dy = p.position.y - self.dock['pose'][1]
        dyaw = yaw_from_q(p.orientation) - self.dock['pose'][2]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        return math.hypot(dx, dy) < self.dock['charge_radius'] and abs(dyaw) < 0.35

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 5.0:
            return
        self.charging = self.at_dock() and self.speed < 0.02
        if self.charging:
            self.level = min(100.0, self.level + self.get_parameter('charge_rate').value * dt)
        else:
            drain = self.get_parameter('drain_idle').value * dt
            drain += self.get_parameter('drain_per_meter').value * self.distance
            self.level = max(0.0, self.level - drain)
        self.distance = 0.0

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.percentage = self.level / 100.0
        msg.voltage = 11.1 + 1.5 * msg.percentage
        msg.present = True
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging
                                   else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.pub.publish(msg)

        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = now.to_msg()
        m.ns, m.id, m.type, m.action = 'battery', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 0.6
        m.scale.z = 0.15
        lvl = self.level / 100.0
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0 - lvl, lvl, 0.1, 1.0)
        m.text = f'{"⚡ " if self.charging else ""}Batarya {self.level:.0f}%'
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(BatterySim())


if __name__ == '__main__':
    main()
