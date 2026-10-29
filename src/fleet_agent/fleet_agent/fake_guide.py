"""가짜 Guide 노드: 카메라 · YOLO 없이 손님 확인 결과를 정해서 돌려준다 (guide_node 대신 띄움)

fleet_agent의 안내 흐름(인사 대기 -> 꺾이는 노드에서 뒤돌기 -> 확인 -> 이어 가기 / guest_lost)을
실제 로봇 · Nav2로 시험할 때, 손님 인식과 상관없이 원하는 결과를 내게 한다.

  ros2 run fleet_agent fake_guide                       # 전부 1초 뒤 OK
  ros2 run fleet_agent fake_guide --lost-at N3          # N3 확인에서 못 찾음 (guest_lost)
  ros2 run fleet_agent fake_guide --greet-fail          # 입구 인사에서 손님 안 옴
  ros2 run fleet_agent fake_guide --ask                 # 요청마다 터미널에서 y/n (사람이 직접 판정)
  ros2 run fleet_agent fake_guide --delay 3 --effects   # 3초 뒤 OK + LCD/LED/부저 효과도
  launch: fleet_agent.launch.xml / robot_system.launch.xml  fake_guide:=true guide_lost_at:=N3 [guide_greet_fail:=true]
          (launch는 ROS 파라미터 lost_at / greet_fail / delay 로 넘김. --ask는 터미널이 필요해서 ros2 run으로만)

주의: 진짜 guide_node와 같이 띄우지 않는다 (둘 다 /guide/result를 보냄). use_guide:=false 로.
"""
import argparse
import json
import sys
import threading
import time


class FakeRules:
    """요청 -> (ok, 기다릴 초, detail). ROS 없이 시험 가능"""

    def __init__(self, lost_at=(), greet_fail=False, delay=1.0, fail_delay=None, lost_once=True):
        self.lost_at = set(lost_at)
        self.greet_fail = greet_fail
        self.delay = delay
        self.fail_delay = delay if fail_delay is None else fail_delay
        self.lost_once = lost_once        # 같은 노드에서는 한 번만 못 찾음 (다시 안내하면 통과)
        self.used = set()

    def decide(self, action, node):
        if action == 'greet' and self.greet_fail and 'greet' not in self.used:
            if self.lost_once:
                self.used.add('greet')
            return False, self.fail_delay, 'not_seen'
        if action == 'watch' and node in self.lost_at and node not in self.used:
            if self.lost_once:
                self.used.add(node)
            return False, self.fail_delay, 'not_seen'
        return True, self.delay, 'guest_seen'


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    ap = argparse.ArgumentParser(description='가짜 Guide 노드 (카메라 없이 손님 확인 결과를 정함)')
    ap.add_argument('--lost-at', default='', help='이 노드(들)의 watch에서 못 찾음. 예: N3 또는 N1,N3')
    ap.add_argument('--greet-fail', action='store_true', help='greet(입구 인사)에서 못 찾음')
    ap.add_argument('--delay', type=float, default=1.0, help='OK까지 초')
    ap.add_argument('--fail-delay', type=float, default=None, help='못 찾음까지 초 (기본 = --delay)')
    ap.add_argument('--always', action='store_true', help='같은 노드에서 매번 못 찾음 (기본: 한 번만)')
    ap.add_argument('--ask', action='store_true', help='요청마다 터미널에서 y/n')
    ap.add_argument('--effects', action='store_true', help='진짜 효과(LCD/LED/부저)도 냄')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('guide')                   # 진짜 guide 노드와 같은 이름 (둘 다 띄우지 말 것)
    # launch에서 오는 ROS 파라미터가 있으면 그쪽을 씀
    lost_p = node.declare_parameter('lost_at', '').value
    greet_p = node.declare_parameter('greet_fail', False).value
    delay_p = node.declare_parameter('delay', -1.0).value
    if lost_p:
        a.lost_at = str(lost_p)
    if greet_p:
        a.greet_fail = True
    if delay_p is not None and float(delay_p) >= 0:
        a.delay = float(delay_p)
    rules = FakeRules(filter(None, a.lost_at.split(',')), a.greet_fail, a.delay, a.fail_delay, not a.always)
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, '/guide/result', rel)
    log = node.get_logger()
    lock = threading.Lock()
    state = {'job': None}                  # (id, cancel_event)

    def effect(kind):
        if a.effects:
            from .guide_effects import play
            threading.Thread(target=play, args=(kind, log.info), daemon=True).start()
        log.info(f'(효과 {kind})')

    def reply(rid, ok, detail):
        pub.publish(String(data=json.dumps({'id': rid, 'ok': ok, 'detail': detail})))
        log.info(f'결과 id {rid}: {"OK" if ok else "못 찾음"} ({detail})')

    def worker(rid, action, nid, cancel):
        if a.ask:
            ans = input(f'\n[{action} @ {nid}] 손님 보임? (y/n) > ').strip().lower()
            ok, detail = ans.startswith('y'), 'ask'
            wait = 0.0
        else:
            ok, wait, detail = rules.decide(action, nid)
        if cancel.wait(wait):
            log.info(f'id {rid} 취소됨 -> 결과 안 보냄 (진짜 guide_node와 같음)')
            return
        reply(rid, ok, detail)

    def on_request(msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        rid, action, nid = req.get('id'), req.get('action'), req.get('node', '')
        log.info(f'요청 id {rid}: {action} @ {nid}')
        with lock:
            if state['job']:
                state['job'][1].set()
            if action == 'arrive':
                effect('arrived')
                return
            if action not in ('greet', 'watch'):
                return
            if action == 'greet':
                effect('welcome')                  # 진짜 guide_node(welcome_on_request)처럼 요청 오자마자 환영
            cancel = threading.Event()
            state['job'] = (rid, cancel)
        threading.Thread(target=worker, args=(rid, action, nid, cancel), daemon=True).start()

    def on_cancel(msg):
        try:
            rid = json.loads(msg.data).get('id')
        except ValueError:
            return
        with lock:
            if state['job'] and state['job'][0] == rid:
                state['job'][1].set()

    node.create_subscription(String, '/guide/request', on_request, rel)
    node.create_subscription(String, '/guide/cancel', on_cancel, rel)
    log.info(f'가짜 guide 시작: 못 찾음 노드 {sorted(rules.lost_at) or "-"}, 인사 실패 {a.greet_fail}, '
             f'{"터미널 y/n" if a.ask else f"{a.delay}초 뒤 결과"}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
