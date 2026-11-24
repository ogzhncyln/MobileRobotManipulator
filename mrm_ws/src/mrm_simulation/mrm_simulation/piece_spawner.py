#!/usr/bin/env python3
"""
Rastgele renkli parça (küp) üretici.

Her odaya `pieces_per_room` adet küp atar. Konumlar:
  * duvarlardan `wall_margin` uzakta,
  * world.yaml'daki yasak bölgelerin (kutular, kapılar, dock, oda merkezi) dışında,
  * birbirinden en az `min_separation` uzakta
olacak şekilde reddetme örneklemesi (rejection sampling) ile seçilir.

Gerçek konumlar (ground truth) /mrm/ground_truth_pieces konusuna JSON olarak
"latched" yayınlanır. Bu SADECE değerlendirme içindir; robot parçaları kamerayla bulur.
"""

import json
import math
import os
import random

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

COLORS_RGB = {'red': (0.90, 0.05, 0.05), 'green': (0.05, 0.75, 0.10), 'blue': (0.05, 0.15, 0.90)}


def piece_sdf(name, color, size):
    r, g, b = COLORS_RGB[color]
    m = 0.03
    i = m * size * size / 6.0
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><link name="link">
  <inertial><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
    <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
  <collision name="c"><geometry><box><size>{size} {size} {size}</size></box></geometry>
    <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
      <contact><ode><kp>1e5</kp><kd>10</kd><min_depth>0.001</min_depth><max_vel>0.1</max_vel></ode></contact>
    </surface></collision>
  <visual name="v"><geometry><box><size>{size} {size} {size}</size></box></geometry>
    <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular></material></visual>
</link></model></sdf>'''


class PieceSpawner(Node):

    def __init__(self):
        super().__init__('piece_spawner')
        default_cfg = os.path.join(get_package_share_directory('mrm_simulation'), 'config', 'world.yaml')
        self.declare_parameter('world_config', default_cfg)
        self.declare_parameter('pieces_per_room', 3)
        self.declare_parameter('seed', -1)          # -1: her çalıştırmada farklı
        self.declare_parameter('rooms', ['room1', 'room2', 'room3', 'room4'])

        with open(self.get_parameter('world_config').value) as f:
            self.world = yaml.safe_load(f)
        seed = self.get_parameter('seed').value
        self.rng = random.Random(None if seed < 0 else seed)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gt_pub = self.create_publisher(String, '/mrm/ground_truth_pieces', qos)
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

    def sample_room(self, room, n):
        r = self.world['rooms'][room]
        x0, x1, y0, y1 = r['bounds']
        m = self.world['spawn']['wall_margin']
        sep = self.world['spawn']['min_separation']
        pts = []
        for _ in range(5000):
            if len(pts) == n:
                break
            x = self.rng.uniform(x0 + m, x1 - m)
            y = self.rng.uniform(y0 + m, y1 - m)
            if any(math.hypot(x - kx, y - ky) < kr for kx, ky, kr in r['keepout']):
                continue
            if any(math.hypot(x - px, y - py) < sep for px, py in pts):
                continue
            pts.append((x, y))
        if len(pts) < n:
            self.get_logger().warn(f'{room}: sadece {len(pts)}/{n} konum bulunabildi')
        return pts

    def run(self):
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/spawn_entity bekleniyor...')
        n = self.get_parameter('pieces_per_room').value
        size = self.world['piece_size']
        colors = self.world['colors']
        truth = []
        for room in self.get_parameter('rooms').value:
            for k, (x, y) in enumerate(self.sample_room(room, n)):
                color = self.rng.choice(colors)
                name = f'piece_{room}_{k}_{color}'
                req = SpawnEntity.Request()
                req.name = name
                req.xml = piece_sdf(name, color, size)
                req.initial_pose = Pose()
                req.initial_pose.position.x = x
                req.initial_pose.position.y = y
                req.initial_pose.position.z = size / 2 + 0.002
                yaw = self.rng.uniform(-math.pi, math.pi)
                req.initial_pose.orientation.z = math.sin(yaw / 2)
                req.initial_pose.orientation.w = math.cos(yaw / 2)
                fut = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
                ok = fut.result() is not None and fut.result().success
                self.get_logger().info(f'{"OK " if ok else "HATA"} {name} @ ({x:.2f}, {y:.2f})')
                truth.append({'name': name, 'room': room, 'color': color,
                              'x': round(x, 3), 'y': round(y, 3)})
        self.gt_pub.publish(String(data=json.dumps(truth)))
        self.get_logger().info(f'{len(truth)} parca uretildi.')


def main():
    rclpy.init()
    node = PieceSpawner()
    node.run()
    # Latched yayın ayakta kalsın diye düğüm açık bekler
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
