"""안내 주행 시험 (실제 로봇, 서버 없이): 서버 대신 안내 Move를 보내고 안내 흐름을 시간순으로 찍는다

main_service를 끄고, 로봇에서 fleet_agent(+ guide_node 또는 fake_guide)를 띄운 뒤 같은 ROS_DOMAIN_ID에서 실행.
서버와 같은 길 · yaw · 끊어 보내기(lookahead)로 보낸다 (guide_plan). 안내가 끝나면 착석 시간만큼 서 있다가 도킹으로 복귀.

  ros2 run fleet_agent guide_run T3                    # D1 -> T3 안내(N6에서 T3 쪽 봄) -> D1 복귀
  ros2 run fleet_agent guide_run T1 T3 T5              # 차례로 3번
  ros2 run fleet_agent guide_run --all --loops 2       # T1~T6 전부 2바퀴
  ros2 run fleet_agent guide_run T5 --no-return        # 복귀 안 함
  ros2 run fleet_agent guide_run --to N5 --face T3     # 아무 노드로 안내 (코너 하나만 시험할 때)
  ros2 run fleet_agent guide_run T3 --hold N1:5        # N1에 닿고 5초 동안 다음 노드를 안 줌 (lock 경쟁 흉내)
  ros2 run fleet_agent guide_run T3 --pause-at N2:4    # N2 도착 뒤 PAUSE 4초 -> RESUME
  ros2 run fleet_agent guide_run T3 --cancel-at N3     # N3 도착 때 CANCEL (그 자리에 섬)
  ros2 run fleet_agent guide_run T3 --lookahead 0      # 끊지 않고 한 번에 전부
  ros2 run fleet_agent guide_run T3 --plan             # 보낼 Move만 찍고 끝 (로봇 안 움직임)

  - 로봇은 --start 노드(기본 D1)에 있어야 한다 (fleet_agent start_node와 같게)
  - 손님 놓침(guest_lost)이면 그 자리에서 도킹으로 복귀 (서버와 같음)
  - Ctrl+C = CANCEL 보내고 결과표
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass, field

from .guide_plan import (REVERSE, GuideMap, MoveFeeder, guide_plan, load_guide_spots, plan_to, return_plan,
                         wrap)

NODE_REACHED, CMD_DONE, CMD_FAILED, EV_BLOCKED = 0, 1, 2, 4
PAUSE, RESUME, CANCEL = 0, 1, 2
EV_NAME = {NODE_REACHED: 'NODE_REACHED', CMD_DONE: 'CMD_DONE', CMD_FAILED: 'CMD_FAILED', EV_BLOCKED: 'BLOCKED'}


@dataclass
class Run:
    label: str
    plan: object
    result: str = ''               # OK / guest_lost:... / nav2_failed / CANCEL / 시간 초과
    sec: float = 0.0
    greet_sec: float = 0.0         # 인사(손님 기다림) 걸린 초
    checks: list = field(default_factory=list)    # [(노드, 걸린 초, ok, 손님쪽 오차°)]
    arrive: str = ''
    final_yaw_err: float = float('nan')
    back: str = '-'
    blocked: list = field(default_factory=list)


class Runner:
    """시나리오 진행 (ROS를 모른다). io: send_move(MoveSpec), send_control(cmd), log(text), pose() -> (x,y,yaw)|None"""

    def __init__(self, io, g, targets, start='D1', spots=None, lookahead=2, back=True, seat_sec=3.0, gap_sec=2.0,
                 hold=None, pause_at=None, cancel_at=None, timeout_sec=240.0, cmd_base=None):
        self.io = io
        self.g = g
        self.start = start
        self.spots = load_guide_spots() if spots is None else spots
        self.targets = list(targets)    # [('table', 'T3')] 또는 [('node', 'N5', face)]
        self.lookahead = lookahead
        self.back = back
        self.seat_sec = seat_sec
        self.gap_sec = gap_sec
        self.hold = hold                # (노드, 초)
        self.pause_at = pause_at        # (노드, 초)
        self.cancel_at = cancel_at      # 노드
        self.timeout_sec = timeout_sec
        self.cmd_id = cmd_base if cmd_base is not None else int(time.time()) % 100000 * 100
        self.runs = []
        self.feeder = None
        self.stage = 'idle'             # idle / guide / seat / back / gap / done
        self.stage_t = 0.0
        self.now = 0.0
        self.outbox = []                # (보낼 t, MoveSpec)
        self.resume_t = None
        self.here = start
        self.guide_t = None             # 지금 기다리는 guide 요청 (action, node, t, look_err)
        self.fired = set()
        self.stopped = False

    # ---------------------------------------------------------------- 진행
    def begin(self, now):
        self.now = now
        self._next_target()

    def _send(self, specs, delay=0.0):
        for m in specs:
            self.outbox.append((self.now + delay, m))

    def _new_feeder(self, plan):
        self.cmd_id += 1
        self.feeder = MoveFeeder(plan, self.cmd_id, self.lookahead)
        self.fired = set()
        self._send(self.feeder.first())

    def _next_target(self):
        if not self.targets or self.stopped:
            self.stage = 'done'
            return
        t = self.targets.pop(0)
        if t[0] == 'table':
            plan = guide_plan(self.g, self.here, t[1], self.spots)
            label = t[1]
        else:
            plan = plan_to(self.g, self.here, t[1], True, t[2] if len(t) > 2 else '')
            label = t[1]
        run = Run(label, plan)
        self.runs.append(run)
        self.io.log(f'==== 안내 {label}: {plan.text()}  (도착지 {plan.goal}{", " + plan.face + " 쪽 봄" if plan.face else ""})')
        self.stage = 'guide'
        self.stage_t = self.now
        self._new_feeder(plan)

    def _go_back(self):
        if not self.back or self.here == self.start:
            self.stage = 'gap'
            self.stage_t = self.now
            return
        plan = return_plan(self.g, self.here, self.start)
        self.io.log(f'---- 복귀: {plan.text()}')
        self.stage = 'back'
        self.stage_t = self.now
        self._new_feeder(plan)

    def tick(self, now):
        self.now = now
        due = [m for at, m in self.outbox if at <= now]
        self.outbox = [(at, m) for at, m in self.outbox if at > now]
        for m in due:
            self.io.log(f'→ {m.text()}')
            self.io.send_move(m)
        if self.resume_t is not None and now >= self.resume_t:
            self.resume_t = None
            self.io.log('! RESUME')
            self.io.send_control(RESUME)
        if self.stage in ('guide', 'back') and now - self.stage_t > self.timeout_sec:
            self.io.log(f'시간 초과 ({self.timeout_sec:.0f}초) -> CANCEL')
            self.io.send_control(CANCEL)
            if self.stage == 'guide':
                self.runs[-1].result = '시간 초과'
            else:
                self.runs[-1].back = '시간 초과'
            self.stopped = True
            self.stage = 'done'
        elif self.stage == 'seat' and now - self.stage_t >= self.seat_sec:
            self._go_back()
        elif self.stage == 'gap' and now - self.stage_t >= self.gap_sec:
            self._next_target()

    # ---------------------------------------------------------------- 로봇 ->
    def on_event(self, etype, cmd_id, node, reason):
        if self.feeder is None or cmd_id != self.feeder.cmd_id:
            return
        run = self.runs[-1] if self.runs else None
        el = self.now - self.stage_t
        self.io.log(f'  {el:6.1f}s {EV_NAME.get(etype, etype)} {node} {reason}'.rstrip())
        if etype == NODE_REACHED:
            self.here = node
            delay = 0.0
            if self.stage == 'guide' and self.hold and node == self.hold[0] and ('hold', node) not in self.fired:
                self.fired.add(('hold', node))
                delay = self.hold[1]
                self.io.log(f'! {node}: 다음 노드 {delay:.0f}초 늦게 줌 (lock 경쟁 흉내)')
            self._send(self.feeder.on_reached(node), delay)
            if self.stage == 'guide' and self.pause_at and node == self.pause_at[0] and ('pause', node) not in self.fired:
                self.fired.add(('pause', node))
                self.io.log(f'! PAUSE {self.pause_at[1]:.0f}초')
                self.io.send_control(PAUSE)
                self.resume_t = self.now + self.pause_at[1]
            if self.stage == 'guide' and self.cancel_at and node == self.cancel_at and ('cancel', node) not in self.fired:
                self.fired.add(('cancel', node))
                self.io.log('! CANCEL (그 자리에 섬)')
                self.io.send_control(CANCEL)
                run.result = 'CANCEL'
                run.sec = el
                self.outbox.clear()
                self.feeder = None
                self.stage = 'seat'
                self.stage_t = self.now
        elif etype == CMD_DONE and node == self.feeder.plan.goal:
            self.here = node
            if self.stage == 'guide':
                run.result = 'OK'
                run.sec = el
                p = self.io.pose()
                if p and run.plan.face:
                    run.final_yaw_err = abs(math.degrees(wrap(self.g.heading(node, run.plan.face) - p[2])))
                self.io.log(f'  안내 도착 {node} ({el:.1f}초). 착석 {self.seat_sec:.0f}초 기다림')
                self.stage = 'seat'
                self.stage_t = self.now
            elif self.stage == 'back':
                run.back = 'OK'
                self.stage = 'gap'
                self.stage_t = self.now
        elif etype == CMD_FAILED:
            if self.stage == 'guide':
                run.result = reason or 'CMD_FAILED'
                run.sec = el
                if reason.startswith('guest_lost'):
                    self.io.log('  손님 놓침 -> 그 자리에서 복귀 (서버와 같음)')
                    self.stage = 'seat'
                    self.stage_t = self.now - self.seat_sec + 1.0
                else:
                    self.io.log('  안내 실패 (손님 놓침 아님) -> 시험 멈춤. fleet_agent 로그 확인')
                    self.stopped = True
                    self.stage = 'done'
            elif self.stage == 'back':
                run.back = reason or '실패'
                self.stopped = True
                self.stage = 'done'
        elif etype == EV_BLOCKED and run:
            run.blocked.append(reason)
            self.io.log('  (서버라면 여기서 우회를 찾는다. 시험에서는 길이 비면 로봇이 알아서 다시 감)')

    def on_guide_request(self, action, node):
        # arrive는 CMD_DONE 바로 뒤에 온다 (그사이 stage가 seat로 바뀜)
        if not self.runs or not (self.stage == 'guide' or (self.stage == 'seat' and action == 'arrive')):
            return
        look = float('nan')
        p = self.io.pose()
        plan = self.runs[-1].plan
        if action == 'watch' and p and node in plan.nodes and plan.nodes.index(node) > 0:
            px, py = self.g.xy(plan.nodes[plan.nodes.index(node) - 1])
            look = abs(math.degrees(wrap(math.atan2(py - p[1], px - p[0]) - p[2])))
        self.guide_t = (action, node, self.now, look)
        extra = '' if math.isnan(look) else f'  손님 쪽과 {look:.0f}° 차이'
        self.io.log(f'  ★ {self.now - self.stage_t:6.1f}s guide {action} @{node}{extra}')
        if action == 'arrive':
            self.runs[-1].arrive = node

    def on_guide_result(self, ok, detail):
        if not self.guide_t or self.stage != 'guide':
            return
        action, node, t0, look = self.guide_t
        self.guide_t = None
        sec = self.now - t0
        self.io.log(f'  ★ guide 결과 {"OK" if ok else "못 찾음"} ({detail}) {sec:.1f}초')
        run = self.runs[-1]
        if action == 'greet':
            run.greet_sec = sec
        else:
            run.checks.append((node, sec, ok, look))

    def done(self):
        return self.stage == 'done'

    def report(self):
        lines = ['', '==== 안내 시험 결과 ====',
                 f'{"대상":5s} {"도착":4s} {"결과":28s} {"초":>6s} {"인사초":>6s} {"확인 (노드 초 손님쪽°)":28s} '
                 f'{"도착효과":6s} {"yaw°":>5s} 복귀']
        for r in self.runs:
            ch = ' '.join(f'{n}:{s:.0f}s{"" if ok else "✗"}{"" if math.isnan(lk) else f"/{lk:.0f}°"}'
                          for n, s, ok, lk in r.checks) or '-'
            yaw = '' if math.isnan(r.final_yaw_err) else f'{r.final_yaw_err:.0f}'
            lines.append(f'{r.label:5s} {r.plan.goal:4s} {r.result or "-":28s} {r.sec:6.1f} {r.greet_sec:6.1f} '
                         f'{ch:28s} {r.arrive or "-":6s} {yaw:>5s} {r.back}')
            if r.blocked:
                lines.append(f'      BLOCKED {r.blocked}')
        lines.append('손님쪽° = 뒤돌아 확인할 때 로봇 앞과 왔던 노드(손님) 쪽 차이. 크면 카메라에 손님이 안 들어옴')
        lines.append('yaw° = 안내 도착 뒤 테이블 쪽과의 차이 (TF 기준)')
        return '\n'.join(lines)


def parse_targets(a, spots):
    if a.to:
        return [('node', a.to, a.face)]
    tables = sorted(spots) if a.all else a.tables
    if not tables:
        raise SystemExit('안내할 테이블을 적거나 --all / --to 노드')
    return [('table', t) for t in tables] * max(1, a.loops)


def node_time(s):
    n, sec = s.split(':')
    return n, float(sec)


def main():
    from rclpy.utilities import remove_ros_args
    ap = argparse.ArgumentParser(description='안내 주행 시험 (서버 없이 안내 Move를 보냄)')
    ap.add_argument('tables', nargs='*', help='안내할 테이블 (T1~T6)')
    ap.add_argument('--all', action='store_true', help='guide_spots의 테이블 전부')
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--to', default='', help='테이블 대신 이 노드로 안내')
    ap.add_argument('--face', default='', help='--to 도착지에서 이 노드 쪽을 봄')
    ap.add_argument('--start', default='D1', help='로봇이 지금 선 노드 = 복귀할 도킹 (기본 D1)')
    ap.add_argument('--lookahead', type=int, default=2, help='앞서 보내는 노드 수 (서버 lock_lookahead, 0 = 전부)')
    ap.add_argument('--no-return', action='store_true')
    ap.add_argument('--seat', type=float, default=3.0, help='안내 도착 뒤 서 있는 초 (착석 흉내)')
    ap.add_argument('--gap', type=float, default=2.0, help='복귀 뒤 다음 안내까지 초')
    ap.add_argument('--hold', type=node_time, default=None, help='노드:초  그 노드에 닿은 뒤 다음 노드를 늦게 줌')
    ap.add_argument('--pause-at', type=node_time, default=None, help='노드:초  PAUSE 뒤 RESUME')
    ap.add_argument('--cancel-at', default='', help='이 노드 도착 때 CANCEL')
    ap.add_argument('--timeout', type=float, default=240.0, help='한 번 이동 최대 초')
    ap.add_argument('--plan', action='store_true', help='보낼 Move만 찍고 끝')
    ap.add_argument('--graph', default='')
    ap.add_argument('--server-config', default='', help='main_service.yaml (guide_spots)')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    g = GuideMap(a.graph)
    spots = load_guide_spots(a.server_config)
    targets = parse_targets(a, spots)
    for n in [a.start, a.to, a.face, a.cancel_at] + [a.hold[0] if a.hold else '', a.pause_at[0] if a.pause_at else '']:
        if n and n not in g.nodes:
            raise SystemExit(f'node_graph에 없는 노드: {n}')

    if a.plan:
        here = a.start
        for t in targets:
            p = guide_plan(g, here, t[1], spots) if t[0] == 'table' else plan_to(g, here, t[1], True, a.face)
            f = MoveFeeder(p, 1, a.lookahead)
            print(f'안내 {t[1]}: {p.text()}  (도착지 {p.goal}, 끝 yaw {math.degrees(p.wps[-1].yaw):.0f}°)')
            for m in f.first():
                print('  ' + m.text())
            for w in p.wps:
                for m in f.on_reached(w.node):
                    print(f'  ({w.node} 도착 뒤) ' + m.text())
            if not a.no_return:
                b = return_plan(g, p.goal, a.start)
                print(f'  복귀: {b.text()}  {"(마지막은 후진 진입)" if b.wps[-1].drive == REVERSE else ""}')
            here = a.start if not a.no_return else p.goal
        return 0

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    import json
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    from fleet_msgs.msg import Control, Move, RobotEvent, Waypoint

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('guide_run')
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    move_pub = node.create_publisher(Move, '/fleet/move', rel)
    ctl_pub = node.create_publisher(Control, '/fleet/control', rel)
    tf = Buffer()
    TransformListener(tf, node)
    t_start = time.monotonic()

    class Io:
        def send_move(self, m):
            msg = Move()
            msg.cmd_id = m.cmd_id
            msg.mode = Move.APPEND if m.append else Move.REPLACE
            msg.drive = Move.REVERSE if m.drive == REVERSE else Move.FORWARD
            msg.goal_node = m.goal
            msg.guide = m.guide
            for nid, x, y, yaw in m.waypoints:
                w = Waypoint()
                w.node_id, w.x, w.y, w.yaw = nid, float(x), float(y), float(yaw)
                msg.waypoints.append(w)
            move_pub.publish(msg)

        def send_control(self, cmd):
            c = Control()
            c.command = cmd
            ctl_pub.publish(c)

        def log(self, text):
            print(f'[{time.monotonic() - t_start:7.1f}] {text}', flush=True)

        def pose(self):
            try:
                t = tf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.0))
            except Exception:  # noqa: BLE001
                return None
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    runner = Runner(Io(), g, targets, a.start, spots, a.lookahead, not a.no_return, a.seat, a.gap,
                    a.hold, a.pause_at, a.cancel_at or None, a.timeout)
    node.create_subscription(RobotEvent, '/fleet/event',
                             lambda e: runner.on_event(e.type, e.cmd_id, e.node_id, e.reason), rel)

    def on_greq(m):
        try:
            r = json.loads(m.data)
        except ValueError:
            return
        runner.on_guide_request(r.get('action', ''), r.get('node', ''))

    def on_gres(m):
        try:
            r = json.loads(m.data)
        except ValueError:
            return
        runner.on_guide_result(bool(r.get('ok')), r.get('detail', ''))

    # fleet_agent가 /fleet/move를 구독할 때까지
    t_end = time.monotonic() + 5.0
    while move_pub.get_subscription_count() == 0 and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if move_pub.get_subscription_count() == 0:
        print('❌ /fleet/move 구독자 없음: fleet_agent가 안 떠 있다 (ROS_DOMAIN_ID, 서버 브릿지 확인)')
        rclpy.shutdown()
        return 1
    # /guide/request는 guide 노드(진짜 또는 fake_guide)가 있을 때만 엿듣는다.
    # fleet_agent는 이 토픽 구독자가 0이면 "guide 노드 없음"으로 손님 확인을 건너뛰는데,
    # 여기서 구독하면 구독자가 생겨 fleet_agent가 오지 않을 결과를 계속 기다리게 된다
    if node.count_subscribers('/guide/request') > 0:
        node.create_subscription(String, '/guide/request', on_greq, rel)
        node.create_subscription(String, '/guide/result', on_gres, rel)
    else:
        print('⚠ guide 노드 없음 (use_guide / fake_guide 안 켬): fleet_agent가 인사 · 확인을 건너뛰고 주행만 한다')
    print(f'로봇이 {a.start}에 있어야 한다. 서버(main_service)는 꺼 둘 것. Ctrl+C = 멈춤 + 결과')
    time.sleep(0.5)
    runner.begin(time.monotonic())
    try:
        while rclpy.ok() and not runner.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            runner.now = time.monotonic()
            runner.tick(runner.now)
    except KeyboardInterrupt:
        Io().send_control(CANCEL)
        time.sleep(0.3)
        if runner.runs and not runner.runs[-1].result:
            runner.runs[-1].result = 'Ctrl+C'
    print(runner.report())
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
