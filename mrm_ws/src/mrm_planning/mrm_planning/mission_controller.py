#!/usr/bin/env python3
"""
GÖREV KONTROLCÜSÜ (Mission Controller)
======================================

Sistemin "beyni". PlanSys2'nin dört bileşenini Python istemcileriyle kullanır:

  DomainExpertClient   -> yüklü PDDL domain'ini okur
  ProblemExpertClient  -> bilgi tabanı: nesneler (instance), olgular (predicate), hedef (goal)
  PlannerClient        -> domain + problem'i POPF'a verip plan alır
  ExecutorClient       -> planı action performer'lara dağıtarak yürütür

Görev akışı (her oda için):
  1. HEDEF: (scanned roomX)            -> plan: [scan]
  2. Beceri sunucusundan bulunan parçaları al, problem'e ekle (piece_at, piece_color)
  3. HEDEF: tüm parçalar aynı renkli kutuda (+ sonraki odada ol)
           -> plan: [pick, place, ..., open_door, move]
  4. Yürütme başarısızsa durumu onar ve YENİDEN PLANLA.

Batarya: /battery_state eşik altına inince (battery_ok robot1) bilgi
tabanından SİLİNİR. Çalışan action bitince, sıradaki action'ın 'at start
(battery_ok)' koşulu tutmaz -> executor planı başarısız sayar -> kontrolcü
yeniden planlar -> POPF şarj içeren yeni bir plan üretir (move.. charge.. move..).
"""

import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from mrm_interfaces.msg import DoorState
from mrm_interfaces.srv import ListPieces
from plansys2_msgs.msg import ActionExecutionInfo
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.ExecutorClient import ExecutorClient
from plansys2_support_py.Parser import Parser
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

ROBOT = 'robot1'


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        cfg = os.path.join(get_package_share_directory('mrm_simulation'), 'config', 'world.yaml')
        self.declare_parameter('world_config', cfg)
        self.declare_parameter('low_battery', 30.0)       # % - bunun altında şarja git
        self.declare_parameter('max_attempts', 8)         # bir hedef için en fazla planlama
        self.declare_parameter('max_piece_failures', 2)   # bu kadar başarısız pick -> parçadan vazgeç
        self.declare_parameter('report_file', os.path.expanduser('~/mrm_mission_report.json'))
        with open(self.get_parameter('world_config').value) as f:
            self.world = yaml.safe_load(f)

        self.battery = None
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
        self.doors_open = set()
        self.create_subscription(DoorState, '/mrm/doors', self.on_door, 10)
        self.scanned_rooms = set()
        self.last_target_room = None
        self.pieces_cli = self.create_client(ListPieces, '/mrm/list_pieces')

        self.ps_domain = DomainExpertClient(self)
        self.ps_problem = ProblemExpertClient(self)
        self.ps_planner = PlannerClient(self)
        self.ps_executor = ExecutorClient(self)

        self.active_pieces = {}       # name -> (color, room)
        self.piece_failures = {}
        self.stats = {'start': time.time(), 'plans': 0, 'replans': 0, 'charges': 0,
                      'battery_events': 0, 'rooms': {}, 'placed': [], 'abandoned': [],
                      'plans_log': []}
        self.battery_lock = threading.Lock()
        self.create_timer(1.0, self.battery_watch)

    # ---------------------------------------------------------------- batarya
    def on_battery(self, msg):
        self.battery = msg

    def on_door(self, msg):
        if msg.open:
            self.doors_open.add(msg.name)

    def battery_watch(self):
        """Batarya eşik altındaysa bilgi tabanından (battery_ok robot1)'i sil."""
        if self.battery is None:
            return
        pct = self.battery.percentage * 100
        charging = self.battery.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if pct < self.get_parameter('low_battery').value and not charging:
            with self.battery_lock:
                pred = Parser.node_from_string_predicate(f'(battery_ok {ROBOT})')
                if self.ps_problem.exist_problem_predicate(pred):
                    self.ps_problem.remove_problem_predicate(pred)
                    self.stats['battery_events'] += 1
                    self.get_logger().warn(
                        f'*** BATARYA DUSUK ({pct:.0f}%) -> (battery_ok {ROBOT}) silindi. '
                        'Siradaki action baslayamayacak ve yeniden planlanacak. ***')

    # --------------------------------------------------------------- bilgi tabanı
    def add_instance(self, name, typ):
        self.ps_problem.add_problem_instance(Parser.param_from_string(name, typ))

    def add_fact(self, fact):
        self.ps_problem.add_problem_predicate(Parser.node_from_string_predicate(fact))

    def remove_fact(self, fact):
        self.ps_problem.remove_problem_predicate(Parser.node_from_string_predicate(fact))

    def has_fact(self, fact):
        return bool(self.ps_problem.exist_problem_predicate(Parser.node_from_string_predicate(fact)))

    def init_knowledge(self):
        """Haritadan bilinen statik bilgiyi PlanSys2 bilgi tabanına yükle."""
        w = self.world
        self.ps_problem.clear_problem_knowledge()
        self.add_instance(ROBOT, 'robot')
        for r in w['rooms']:
            self.add_instance(r, 'room')
        for c in w['colors']:
            self.add_instance(c, 'color')
        self.add_instance(w['dock']['name'], 'dock')
        self.add_fact(f'(dock_at {w["dock"]["name"]} {w["dock"]["room"]})')
        for d, info in w['doors'].items():
            self.add_instance(d, 'door')
            a, b = info['rooms']
            self.add_fact(f'(connected {a} {b} {d})')
            self.add_fact(f'(connected {b} {a} {d})')
            self.add_fact(f'(button_side {d} {info["approach_from"]})')
            self.add_fact(f'(door_closed {d})')
        for b, info in w['bins'].items():
            self.add_instance(b, 'bin')
            self.add_fact(f'(bin_at {b} {info["room"]})')
            self.add_fact(f'(bin_color {b} {info["color"]})')
        self.add_fact(f'(robot_at {ROBOT} {w["room_order"][0]})')
        self.add_fact(f'(hand_free {ROBOT})')
        self.add_fact(f'(battery_ok {ROBOT})')
        self.add_fact(f'(robot_free {ROBOT})')
        self.get_logger().info('Bilgi tabani yuklendi:\n' + (self.ps_problem.get_problem() or ''))

    def ensure_knowledge(self):
        """
        PlanSys2 süreci çöküp yeniden başlarsa (launch'ta respawn=true) bilgi tabanı
        BOŞ gelir. Bunu fark et ve kontrolcünün kendi kayıtlarından yeniden kur.
        """
        if self.ps_problem.get_problem_instance(ROBOT) is not None:
            return
        self.get_logger().error('Bilgi tabani kaybolmus (PlanSys2 yeniden basladi?) -> yeniden kuruluyor')
        self.init_knowledge()
        for d in self.doors_open:
            self.remove_fact(f'(door_closed {d})')
            self.add_fact(f'(door_open {d})')
        room = self.last_target_room or self.world['room_order'][0]
        self.remove_fact(f'(robot_at {ROBOT} {self.world["room_order"][0]})')
        self.add_fact(f'(robot_at {ROBOT} {room})')
        for r in self.scanned_rooms:
            self.add_fact(f'(scanned {r})')
        placed = set(self.stats['placed'])
        for name, (color, r) in self.active_pieces.items():
            self.add_instance(name, 'piece')
            self.add_fact(f'(piece_color {name} {color})')
            if name in placed:
                self.add_fact(f'(in_bin {name} bin_{r}_{color})')
            else:
                self.add_fact(f'(piece_at {name} {r})')

    def register_pieces(self, room):
        req = ListPieces.Request(room=room)
        fut = self.pieces_cli.call_async(req)
        while not fut.done():
            time.sleep(0.05)
        new = 0
        for p in fut.result().pieces:
            if p.name in self.active_pieces or p.name in self.stats['abandoned']:
                continue
            new += 1
            self.add_instance(p.name, 'piece')
            self.add_fact(f'(piece_at {p.name} {room})')
            self.add_fact(f'(piece_color {p.name} {p.color})')
            self.active_pieces[p.name] = (p.color, room)
            self.get_logger().info(f'  + parca: {p.name} ({p.color}) @ ({p.position.x:.2f}, {p.position.y:.2f})')
        return new

    def abandon_piece(self, name):
        color, room = self.active_pieces.pop(name)
        for f in [f'(piece_at {name} {room})', f'(piece_color {name} {color})',
                  f'(holding {ROBOT} {name})']:
            self.remove_fact(f)
        self.ps_problem.remove_problem_instance(Parser.param_from_string(name, 'piece'))
        self.stats['abandoned'].append(name)
        self.get_logger().error(f'Parcadan vazgecildi: {name}')

    def repair_state(self):
        """
        Başarısız bir action'dan sonra sembolik durum ile fiziksel durumu uzlaştır.
        Örn. pick başladı (at start: hand_free ve piece_at silindi) ama tutamadı.
        """
        # Yarıda kalan action robot_free kilidini geri vermemiş olabilir
        if not self.has_fact(f'(robot_free {ROBOT})'):
            self.add_fact(f'(robot_free {ROBOT})')
            self.get_logger().info('  onarim: (robot_free robot1) geri eklendi')
        holding = [p for p in self.active_pieces if self.has_fact(f'(holding {ROBOT} {p})')]
        if not holding and not self.has_fact(f'(hand_free {ROBOT})'):
            self.add_fact(f'(hand_free {ROBOT})')
            self.get_logger().info('  onarim: (hand_free robot1) geri eklendi')
        for name, (color, room) in self.active_pieces.items():
            in_bin = any(self.has_fact(f'(in_bin {name} {b})') for b in self.world['bins'])
            if not in_bin and name not in holding and not self.has_fact(f'(piece_at {name} {room})'):
                self.add_fact(f'(piece_at {name} {room})')
                self.get_logger().info(f'  onarim: (piece_at {name} {room}) geri eklendi')
        for d in self.world['doors']:
            if not self.has_fact(f'(door_open {d})') and not self.has_fact(f'(door_closed {d})'):
                self.add_fact(f'(door_closed {d})')
        # robot_at kaybolduysa (move yarıda kaldıysa) en yakın odayı bul: beceri sunucusu
        # her zaman bir odada durur; burada sembolik olarak son bilinen hedef odayı kullanırız
        if not any(self.has_fact(f'(robot_at {ROBOT} {r})') for r in self.world['rooms']):
            room = self.last_target_room
            self.add_fact(f'(robot_at {ROBOT} {room})')
            self.get_logger().info(f'  onarim: (robot_at robot1 {room})')

    # ------------------------------------------------------------ plan + yürüt
    def achieve(self, goal_fn, label):
        """goal_fn() -> PDDL hedef metni. Hedefe ulaşana kadar planla-yürüt-onar."""
        for attempt in range(1, self.get_parameter('max_attempts').value + 1):
            goal = goal_fn()
            if goal is None:
                return True
            self.ensure_knowledge()
            self.ps_problem.remove_problem_goal()
            self.ps_problem.add_problem_goal(Parser.tree_from_string(goal))
            if self.ps_problem.is_problem_goal_satisfied(Parser.tree_from_string(goal)):
                self.get_logger().info(f'[{label}] hedef zaten saglaniyor')
                return True
            plan = self.ps_planner.get_plan(self.ps_domain.get_domain(), self.ps_problem.get_problem())
            self.stats['plans'] += 1
            if attempt > 1:
                self.stats['replans'] += 1
            if plan is None:
                self.get_logger().error(f'[{label}] PLAN BULUNAMADI (deneme {attempt})')
                time.sleep(2.0)
                continue
            lines = [f'    {it.time:7.1f}: {it.action}  [{it.duration:.0f}]' for it in plan.items]
            self.get_logger().info(f'[{label}] PLAN (deneme {attempt}, {len(lines)} adim):\n' + '\n'.join(lines))
            self.stats['plans_log'].append({'label': label, 'attempt': attempt,
                                            'actions': [it.action for it in plan.items]})
            if not self.ps_executor.start_plan_execution(plan):
                time.sleep(2.0)
                continue
            last = ''
            while self.ps_executor.execute_and_check_plan(verbose=False):
                fb = self.ps_executor.get_feedback()
                if fb is not None:
                    running = [a for a in fb.feedback.action_execution_status
                               if a.status == ActionExecutionInfo.EXECUTING]
                    txt = ', '.join(f'{a.action_full_name}' for a in running)
                    if txt and txt != last:
                        self.get_logger().info(f'  >> calisiyor: {txt}')
                        last = txt
                time.sleep(0.5)
            result = self.ps_executor.get_result()
            if result is not None and result.result == result.SUCCESS:
                self.get_logger().info(f'[{label}] PLAN BASARIYLA TAMAMLANDI')
                self.collect_stats(result)
                return True
            self.handle_failure(result, label)
        return False

    def collect_stats(self, result):
        for a in result.action_execution_status:
            if a.status == ActionExecutionInfo.SUCCEEDED:
                if a.action == 'place':
                    self.stats['placed'].append(a.arguments[1])
                elif a.action == 'charge':
                    self.stats['charges'] += 1

    def handle_failure(self, result, label):
        failed = []
        if result is not None:
            self.collect_stats(result)
            failed = [a for a in result.action_execution_status
                      if a.status in (ActionExecutionInfo.FAILED, ActionExecutionInfo.CANCELLED)]
        names = ', '.join(a.action_full_name for a in failed) or '(bilinmiyor)'
        self.get_logger().warn(f'[{label}] PLAN BASARISIZ: {names} -> durum onariliyor, yeniden planlanacak')
        # Batarya kesintisinde sıradaki action 'at start (battery_ok)' sağlanmadığı için
        # başlayamaz ve başarısız görünür: bu bir KAVRAMA hatası değildir, sayma.
        battery_interrupt = not self.has_fact(f'(battery_ok {ROBOT})')
        if battery_interrupt:
            self.get_logger().warn(f'[{label}] Sebep: batarya kesintisi -> sarj iceren yeni plan bekleniyor')
        for a in failed:
            if battery_interrupt:
                break
            # pick VEYA place tekrar tekrar başarısız oluyorsa (örn. sembolik durum fiziksel
            # dünyayla uyuşmuyorsa) aynı action'ı sonsuza kadar denemek yerine parçadan vazgeç
            if a.action in ('pick', 'place') and a.status == ActionExecutionInfo.FAILED:
                p = a.arguments[1]
                self.piece_failures[p] = self.piece_failures.get(p, 0) + 1
                if self.piece_failures[p] >= self.get_parameter('max_piece_failures').value \
                        and p in self.active_pieces:
                    self.abandon_piece(p)
        self.repair_state()
        time.sleep(1.0)

    # -------------------------------------------------------------------- görev
    def run(self):
        self.get_logger().info('PlanSys2 bilesenleri bekleniyor...')
        self.ps_problem.wait_active()
        self.ps_planner.wait_active()
        self.ps_executor.wait_active()
        while not self.pieces_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Beceri sunucusu bekleniyor...')
        self.init_knowledge()

        order = self.world['room_order']
        bins = self.world['bins']
        for i, room in enumerate(order):
            t_room = time.time()
            nxt = order[i + 1] if i + 1 < len(order) else None
            self.last_target_room = room
            self.get_logger().info(f'\n{"=" * 60}\n ODA {i+1}/{len(order)}: {room}\n{"=" * 60}')

            # Faz 1: tara
            self.achieve(lambda: f'(and (scanned {room}))', f'{room}/tarama')
            self.scanned_rooms.add(room)
            self.register_pieces(room)

            # Faz 2: ayıkla. Ardından DOĞRULAMA taraması (farklı bakış noktalarından): tek
            # noktadan kaçan ya da robot tarafından itilmiş küpler varsa onları da ayıkla.
            def sort_goal(room=room):
                parts = [f'(in_bin {p} bin_{room}_{c})'
                         for p, (c, r) in self.active_pieces.items() if r == room]
                return f'(and {" ".join(parts)})' if parts else None
            ok = self.achieve(sort_goal, f'{room}/ayiklama')
            for v in range(2):
                if not ok:
                    break
                self.remove_fact(f'(scanned {room})')
                self.achieve(lambda: f'(and (scanned {room}))', f'{room}/dogrulama{v+1}')
                new = self.register_pieces(room)
                self.get_logger().info(f'[{room}] dogrulama taramasi {v+1}: {new} yeni parca')
                self.stats.setdefault('verification_new', 0)
                self.stats['verification_new'] += new
                if new == 0:
                    break
                ok = self.achieve(sort_goal, f'{room}/ayiklama+{v+1}')

            # Faz 3: sonraki odaya geç (kapıyı aç + hareket)
            self.last_target_room = nxt or room
            if ok and nxt:
                ok = self.achieve(lambda: f'(and (robot_at {ROBOT} {nxt}))', f'{room}->{nxt}')
            n_room = sum(1 for _, (c, r) in self.active_pieces.items() if r == room)
            self.stats['rooms'][room] = {'pieces_found': n_room, 'success': ok,
                                         'duration_s': round(time.time() - t_room, 1)}
            if not ok:
                self.get_logger().error(f'{room} tamamlanamadi, gorev durduruluyor')
                break

        self.stats['duration_s'] = round(time.time() - self.stats['start'], 1)
        self.stats.pop('start')
        report = self.get_parameter('report_file').value
        with open(report, 'w') as f:
            json.dump(self.stats, f, indent=2, ensure_ascii=False)
        self.get_logger().info(
            f'\n{"#" * 60}\n GÖREV TAMAMLANDI\n  Kutuya konan: {len(self.stats["placed"])}\n'
            f'  Sarj: {self.stats["charges"]}  Yeniden planlama: {self.stats["replans"]}\n'
            f'  Süre: {self.stats["duration_s"]} s\n  Rapor: {report}\n{"#" * 60}')


def main():
    rclpy.init()
    node = MissionController()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
