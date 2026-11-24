#!/usr/bin/env python3
"""
ALT SİMÜLASYON TEST KOŞUCUSU
============================

Tam görev yerine robotun temel becerilerini (yanaşma, kavrama, bırakma) kısa, tekrarlanabilir
senaryolarla dener. Tüm senaryolar TEK bir Gazebo oturumunda koşar (her biri ~1-2 dk):

  ros2 launch mrm_bringup skill_test.launch.py                       # tüm senaryolar
  ros2 launch mrm_bringup skill_test.launch.py scenarios:=pick_basic,pick_blind
  ros2 launch mrm_bringup skill_test.launch.py rtf:=2.0              # 2x hızlı fizik (CPU yeterse)

Her senaryo için:
  1. debug_reset (dünya modeli + tutulan küp sıfırlanır), eski test küpleri silinir
  2. robot Gazebo'da yeniden konumlandırılır, AMCL'ye /initialpose verilir
  3. küpler oluşturulur; dünya modeline 'est_err' kadar hatalı konumla eklenir (tarama atlanır)
  4. adımlar (pick/place...) beceri sunucusuna gönderilir
  5. GERÇEK konumlarla (Gazebo) değerlendirilir: hedef tutuldu mu / kutuda mı,
     yanaşma sırasında hedef ve komşu küpler itildi mi (cm), süre

Sonuç tablosu loga basılır ve JSON olarak 'report' yoluna yazılır.
"""

import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from mrm_interfaces.action import ExecuteSkill
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mrm_simulation.piece_spawner import piece_sdf

PIECE_Z_GROUND = 0.05     # bu yükseklikten alçaktaki küp "yerde"


def wait(fut, timeout):
    t0 = time.monotonic()
    while not fut.done():
        if time.monotonic() - t0 > timeout:
            return None
        time.sleep(0.02)
    return fut.result()


class SkillTest(Node):

    def __init__(self):
        super().__init__('skill_test')
        share = get_package_share_directory('mrm_skills')
        self.declare_parameter('scenario_file', os.path.join(share, 'config', 'skill_scenarios.yaml'))
        self.declare_parameter('scenarios', 'all')
        self.declare_parameter('repeat', 1)
        self.declare_parameter('report', os.path.expanduser('~/.ros/mrm_skill_test_report.json'))
        world_cfg = os.path.join(get_package_share_directory('mrm_simulation'), 'config', 'world.yaml')
        with open(world_cfg) as f:
            self.world = yaml.safe_load(f)
        with open(self.get_parameter('scenario_file').value) as f:
            self.all = yaml.safe_load(f)['scenarios']

        self.skill = ActionClient(self, ExecuteSkill, '/mrm/execute_skill')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.models = None
        self.lock = threading.Lock()
        self.track = {}      # model adı -> dict(start, max_ground_disp, lifted)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 5)

    # ------------------------------------------------------------ yardımcılar
    def on_models(self, msg):
        with self.lock:
            self.models = {n: p for n, p in zip(msg.name, msg.pose)}
            for n, tr in self.track.items():
                p = self.models.get(n)
                if p is None:
                    continue
                if p.position.z > PIECE_Z_GROUND:
                    tr['lifted'] = True
                elif not tr['lifted']:
                    d = math.hypot(p.position.x - tr['start'][0], p.position.y - tr['start'][1])
                    tr['max_ground_disp'] = max(tr['max_ground_disp'], d)

    def model(self, name):
        with self.lock:
            return None if self.models is None else self.models.get(name)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run_skill(self, skill, args, timeout=300.0):
        goal = ExecuteSkill.Goal()
        goal.skill, goal.args = skill, [str(a) for a in args]
        fut = self.skill.send_goal_async(goal)
        gh = wait(fut, 15.0)
        if gh is None or not gh.accepted:
            return False, 'hedef kabul edilmedi'
        res = wait(gh.get_result_async(), timeout)
        if res is None:
            gh.cancel_goal_async()
            return False, 'zaman asimi'
        return bool(res.result.success), res.result.message

    def delete_test_pieces(self):
        with self.lock:
            names = [n for n in (self.models or {}) if n.startswith('piece_')]
        for n in names:
            req = DeleteEntity.Request()
            req.name = n
            wait(self.delete_cli.call_async(req), 10.0)
        with self.lock:
            self.track.clear()

    def teleport(self, x, y, yaw):
        req = SetEntityState.Request()
        st = EntityState()
        st.name = 'mrm_robot'
        st.reference_frame = 'world'
        st.pose.position.x, st.pose.position.y, st.pose.position.z = float(x), float(y), 0.005
        st.pose.orientation.z, st.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req.state = st
        r = wait(self.state_cli.call_async(req), 10.0)
        if r is None or not r.success:
            raise RuntimeError('robot isinlanamadi')
        time.sleep(1.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.0025
        cov[35] = 0.003
        msg.pose.covariance = cov
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            time.sleep(0.5)
        time.sleep(2.0)

    def spawn(self, name, color, x, y):
        size = self.world['piece_size']
        req = SpawnEntity.Request()
        req.name = name
        req.xml = piece_sdf(name, color, size)
        req.initial_pose = Pose()
        req.initial_pose.position.x, req.initial_pose.position.y = float(x), float(y)
        req.initial_pose.position.z = size / 2 + 0.002
        r = wait(self.spawn_cli.call_async(req), 15.0)
        if r is None or not r.success:
            raise RuntimeError(f'kup olusturulamadi: {name}')

    # ------------------------------------------------------------ senaryo
    def run_scenario(self, key, sc):
        self.get_logger().info(f'========== SENARYO {key}: {sc.get("desc", "")}')
        t_wall, t_sim = time.monotonic(), self.sim_now()
        out = {'scenario': key, 'steps': [], 'checks': {}, 'ok': False}
        self.run_skill('debug_reset', [])
        self.delete_test_pieces()
        self.teleport(*sc['robot'])
        names = {}
        for k, pc in sc['pieces'].items():
            gz = f'piece_test_{k}_{pc["color"]}'
            names[k] = gz
            self.spawn(gz, pc['color'], pc['x'], pc['y'])
        time.sleep(1.5)       # küplerin yerleşmesini bekle
        for k, pc in sc['pieces'].items():
            m = self.model(names[k])
            start = (m.position.x, m.position.y) if m else (pc['x'], pc['y'])
            with self.lock:
                self.track[names[k]] = {'start': start, 'max_ground_disp': 0.0, 'lifted': False}
            if pc.get('known', True):
                ex, ey = pc.get('est_err', [0.0, 0.0])
                self.run_skill('debug_add_piece', [k, pc['color'], 'room1', pc['x'] + ex, pc['y'] + ey])
        time.sleep(1.0)

        all_ok = True
        for step in sc['steps']:
            t0 = self.sim_now()
            ok, msg = self.run_skill(step[0], step[1:])
            dt = self.sim_now() - t0
            out['steps'].append({'step': ' '.join(map(str, step)), 'ok': ok, 'msg': msg, 'sim_s': round(dt, 1)})
            self.get_logger().info(f'  adim {" ".join(map(str, step))}: {"OK" if ok else "HATA"} '
                                   f'({dt:.0f} s) {msg}')
            if not ok:
                all_ok = False
                break

        # Değerlendirme (gerçek konumlar)
        time.sleep(1.0)
        for k, exp in sc.get('expect', {}).items():
            m = self.model(names[k])
            tr = self.track.get(names[k], {})
            res = {'expect': exp, 'pre_grasp_push_cm': round(100 * tr.get('max_ground_disp', 0.0), 1)}
            if m is None:
                res['ok'] = False
            elif exp == 'held':
                res['z'] = round(m.position.z, 3)
                res['ok'] = m.position.z > 0.08
            elif exp.startswith('in_bin:'):
                b = self.world['bins'][exp.split(':', 1)[1]]
                dx, dy = m.position.x - b['position'][0], m.position.y - b['position'][1]
                inner = b['size'] / 2 - 0.012
                res['offset_cm'] = [round(100 * dx, 1), round(100 * dy, 1)]
                res['ok'] = abs(dx) < inner and abs(dy) < inner and m.position.z < b['height']
            out['checks'][k] = res
            all_ok = all_ok and res['ok']
        for k in sc.get('no_push', []):
            tr = self.track.get(names[k], {})
            m = self.model(names[k])
            d = math.hypot(m.position.x - tr['start'][0], m.position.y - tr['start'][1]) if m else 9.9
            out['checks'][k] = {'expect': 'no_push', 'push_cm': round(100 * d, 1), 'ok': d < 0.015}
            all_ok = all_ok and d < 0.015
        out['ok'] = all_ok
        out['sim_s'] = round(self.sim_now() - t_sim, 1)
        out['wall_s'] = round(time.monotonic() - t_wall, 1)
        self.get_logger().info(f'  SONUC {key}: {"GECTI" if all_ok else "KALDI"} '
                               f'{json.dumps(out["checks"], ensure_ascii=False)}')
        return out

    def run(self):
        self.get_logger().info('Beceri sunucusu bekleniyor...')
        if not self.skill.wait_for_server(timeout_sec=300.0):
            self.get_logger().error('beceri sunucusu yok')
            return 1
        for cli in (self.spawn_cli, self.delete_cli, self.state_cli):
            cli.wait_for_service(timeout_sec=60.0)
        # Nav2 + AMCL'nin başlatılması: zararsız bir beceriyle (kol home) hazır olduğunu doğrula
        t0 = time.monotonic()
        while time.monotonic() - t0 < 180.0:
            ok, _ = self.run_skill('home_arm', [], timeout=60.0)
            if ok:
                break
            time.sleep(3.0)
        time.sleep(5.0)

        sel = self.get_parameter('scenarios').value
        keys = list(self.all) if sel in ('', 'all') else [s.strip() for s in sel.split(',') if s.strip()]
        results = []
        for r in range(self.get_parameter('repeat').value):
            for k in keys:
                if k not in self.all:
                    self.get_logger().error(f'bilinmeyen senaryo: {k}')
                    continue
                try:
                    res = self.run_scenario(k, self.all[k])
                except Exception as e:  # noqa: BLE001
                    res = {'scenario': k, 'ok': False, 'error': repr(e)}
                    self.get_logger().error(f'  {k}: {e!r}')
                res['run'] = r
                results.append(res)
        self.run_skill('debug_reset', [])

        self.get_logger().info('================ OZET ================')
        for res in results:
            steps = '; '.join(f'{s["step"]}={"ok" if s["ok"] else s["msg"]}({s["sim_s"]}s)'
                              for s in res.get('steps', []))
            self.get_logger().info(f'{"GECTI" if res["ok"] else "KALDI"}  {res["scenario"]:<24} '
                                   f'sim={res.get("sim_s", "-")}s  {steps}  '
                                   f'{json.dumps(res.get("checks", {}), ensure_ascii=False)}')
        n_ok = sum(r['ok'] for r in results)
        self.get_logger().info(f'{n_ok}/{len(results)} senaryo gecti')
        path = self.get_parameter('report').value
        if path:
            with open(path, 'w') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f'Rapor: {path}')
        return 0 if n_ok == len(results) else 2


def main():
    rclpy.init()
    node = SkillTest()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
