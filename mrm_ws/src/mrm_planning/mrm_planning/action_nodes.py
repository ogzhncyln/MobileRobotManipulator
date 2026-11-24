#!/usr/bin/env python3
"""
PlanSys2 ACTION PERFORMER düğümleri.

PlanSys2 executor bir plan çalıştırırken her action için "kim yapabilir?" diye
/actions_hub konusuna REQUEST yayınlar. Bu action'ı yöneten düğüm (action_name
parametresi eşleşen) RESPONSE ile cevap verir, executor CONFIRM gönderir ve düğüm
lifecycle'da ACTIVE durumuna geçer. ACTIVE iken `do_work()` periyodik çağrılır;
düğüm `send_feedback()` ile ilerleme bildirir ve `finish()` ile bitirir.

Her PDDL action'ı burada beceri sunucusundaki bir beceriye çevrilir:

  PDDL                                   Beceri (/mrm/execute_skill)
  (move r from to d)                 ->  navigate_room [to]
  (scan r rm)                        ->  scan_room [rm]
  (pick r p rm)                      ->  pick [p]
  (place r p b rm c)                 ->  place [p, b]
  (open_door r d rm other)           ->  open_door [d]
  (charge r dk rm)                   ->  charge [dk]

Tüm performer'lar tek süreçte, MultiThreadedExecutor ile çalışır.
"""

import rclpy
from lifecycle_msgs.msg import Transition  # noqa: F401
from mrm_interfaces.action import ExecuteSkill
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# action adı -> PDDL argümanlarından beceri adı ve argümanlarını üreten fonksiyon
ACTION_MAP = {
    'move':      lambda a: ('navigate_room', [a[2]]),
    'scan':      lambda a: ('scan_room', [a[1]]),
    'pick':      lambda a: ('pick', [a[1]]),
    'place':     lambda a: ('place', [a[1], a[2]]),
    'open_door': lambda a: ('open_door', [a[1]]),
    'charge':    lambda a: ('charge', [a[1]]),
}


class SkillPerformer(ActionExecutorClient):

    def __init__(self, action_name):
        # rate: do_work periyodu (s)
        super().__init__(f'{action_name}_performer', 0.5)
        self.set_parameters([rclpy.parameter.Parameter('action_name', value=action_name)])
        self.action_name = action_name
        self.cbg = ReentrantCallbackGroup()
        self.client = ActionClient(self, ExecuteSkill, '/mrm/execute_skill', callback_group=self.cbg)
        self._reset()

    def _reset(self):
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.progress = 0.0
        self.status_text = ''

    def on_activate(self, state):
        self._reset()
        return super().on_activate(state)

    def on_deactivate(self, state):
        # Executor planı iptal ettiyse çalışan beceriyi de iptal et
        if self.goal_handle is not None and self.result_future is not None \
                and not self.result_future.done():
            self.get_logger().warn(f'{self.action_name}: beceri iptal ediliyor')
            self.goal_handle.cancel_goal_async()
        return super().on_deactivate(state)

    def on_feedback(self, msg):
        self.progress = msg.feedback.progress
        self.status_text = msg.feedback.status

    def do_work(self):
        # 1) İlk çağrı: beceri hedefini gönder
        if self.send_future is None:
            args = list(self.current_arguments)
            skill, skill_args = ACTION_MAP[self.action_name](args)
            self.get_logger().info(f'PDDL ({self.action_name} {" ".join(args)}) -> {skill} {skill_args}')
            if not self.client.wait_for_server(timeout_sec=2.0):
                self.finish(False, 0.0, 'skill server yok')
                return
            goal = ExecuteSkill.Goal(skill=skill, args=skill_args)
            self.send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
            self.send_feedback(0.0, f'{skill} gonderildi')
            return
        # 2) Hedef kabul edildi mi?
        if self.goal_handle is None:
            if not self.send_future.done():
                return
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.finish(False, 0.0, 'beceri reddedildi (sunucu mesgul)')
                return
            self.result_future = self.goal_handle.get_result_async()
            return
        # 3) Bitti mi?
        if not self.result_future.done():
            self.send_feedback(self.progress, self.status_text)
            return
        res = self.result_future.result().result
        self.get_logger().info(f'{self.action_name} sonuc: {res.success} - {res.message}')
        self.finish(res.success, 1.0 if res.success else self.progress, res.message)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = []
    for name in ACTION_MAP:
        n = SkillPerformer(name)
        n.trigger_configure()   # lifecycle: UNCONFIGURED -> INACTIVE (executor CONFIRM ile ACTIVE yapar)
        executor.add_node(n)
        nodes.append(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
