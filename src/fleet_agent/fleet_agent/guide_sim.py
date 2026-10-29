"""안내 시뮬레이터 (PC, ROS · 로봇 · 서버 없이): 진짜 core.Driver를 진짜 node_graph 위에서 돌려 본다

서버처럼 안내 Move를 끊어 보내고(guide_plan.MoveFeeder), Nav2 · 회전 · Guide 노드(손님)는 흉내 낸다.
로봇에 올리기 전에 "어디서 인사 · 뒤돌아 확인 · 도착 효과가 나오는지, 손님을 놓치면 어떻게 되는지"를 본다.

  python3 -m fleet_agent.guide_sim                     # T1~T6 전부, 요약표
  python3 -m fleet_agent.guide_sim T3 -v               # T3 하나, 시간순 로그
  python3 -m fleet_agent.guide_sim T5 --lost-at N3 -v  # N3 확인에서 손님 못 찾음 -> guest_lost -> 복귀
  python3 -m fleet_agent.guide_sim T3 --greet-fail     # 입구에서 손님이 안 옴
  python3 -m fleet_agent.guide_sim T3 --lookahead 1    # 서버가 1칸씩만 보낼 때 (lock 경쟁)
  python3 -m fleet_agent.guide_sim T3 --pause-at N2:3  # N2 도착 뒤 PAUSE 3초 -> RESUME
  python3 -m fleet_agent.guide_sim T3 --obstacle N2:4  # N2를 지나 4초 동안 사람이 막음
  python3 -m fleet_agent.guide_sim T3 --check-turn 30 --check-gap 0.5   # 확인 기준 바꿔 보기
  (빌드 뒤에는 ros2 run fleet_agent guide_sim ... 도 됨)
"""
import argparse
import math
import sys
from dataclasses import dataclass, field

from .core import (CMD_DONE, CMD_FAILED, EV_BLOCKED, NODE_REACHED, PAUSE, RESUME, BlockPolicy, Driver,
                   ReverseControl, load_precision_stops, wrap)
from .guide_plan import GuideMap, MoveFeeder, guide_plan, load_guide_spots, return_plan

EV_NAME = {NODE_REACHED: 'NODE_REACHED', CMD_DONE: 'CMD_DONE', CMD_FAILED: 'CMD_FAILED', EV_BLOCKED: 'BLOCKED'}


@dataclass
class Guest:
    """손님(= Guide 노드 결과) 흉내"""
    lost_at: set = field(default_factory=set)   # 이 노드의 watch에서 못 찾음
    greet_fail: bool = False                    # 입구 인사에서 손님이 안 옴
    react_sec: float = 1.0                      # 찾을 때 걸리는 시간 (YOLO confirm 2번 ≈ 0.5~1초)
    wait_sec: float = 30.0                      # 못 찾을 때 포기까지 (guide_node wait_sec)
    once: bool = False                          # 한 번 놓친 노드는 다음엔 찾음 (여러 번 안내할 때)
    used: set = field(default_factory=set)

    def answer(self, action, node):
        """-> (ok, 걸리는 초, detail)"""
        key = 'greet' if action == 'greet' else node
        fail = (action == 'greet' and self.greet_fail) or (action == 'watch' and node in self.lost_at)
        if fail and not (self.once and key in self.used):
            self.used.add(key)
            return False, self.wait_sec, 'not_seen'
        return True, self.react_sec, 'guest_seen'


class SimRobot:
    """Driver io. Nav2 = 직선으로 speed m/s, 도착하면 목표 yaw. cmd_vel = 적분"""

    def __init__(self, x, y, yaw, guest, speed=0.15, prec_sec=1.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.speed = speed
        self.nav = None
        self.guest = guest
        self.prec_sec = prec_sec
        self.t = 0.0
        self.driver = None
        self.on_event = None          # (type, cmd, node, reason) -> None
        self.lines = []               # (t, 종류, 글)
        self.events = []              # (t, type, node, reason)
        self.guides = []              # (t, action, node, x, y, yaw)
        self.guide_pending = None     # (끝나는 t, id, ok, detail)
        self.prec_pending = None
        self.guide_id = 0
        self.dist = 0.0

    def log(self, text, kind='log'):
        self.lines.append((self.t, kind, text))

    # ---- Driver io
    def nav_go(self, x, y, yaw):
        self.nav = (x, y, yaw)

    def nav_cancel(self):
        self.nav = None

    def cmd_vel(self, v, w):
        self.v, self.w = v, w

    def event(self, etype, cmd, node, reason):
        self.events.append((self.t, etype, node, reason))
        self.log(f'{EV_NAME.get(etype, etype)} {node} {reason}'.rstrip(), 'event')
        if self.on_event:
            self.on_event(etype, cmd, node, reason)

    def precision(self, req):
        self.log(f'정밀 동작 {req.get("node")}:{req.get("method")}', 'prec')
        self.prec_pending = self.t + self.prec_sec

    def precision_cancel(self):
        self.prec_pending = None

    def guide(self, req):
        self.guide_id += 1
        a, n = req['action'], req['node']
        self.guides.append((self.t, a, n, self.x, self.y, self.yaw))
        self.log(f'guide {a} @{n} (yaw {math.degrees(self.yaw):.0f}°)', 'guide')
        if a in ('greet', 'watch'):
            ok, sec, detail = self.guest.answer(a, n)
            self.guide_pending = (self.t + sec, self.guide_id, ok, detail)

    def guide_cancel(self):
        if self.guide_pending:
            self.log('guide 취소', 'guide')
        self.guide_pending = None

    def direction(self, d):
        pass

    def lcd_override(self, text):
        if text:
            self.log(f'LCD "{text}"', 'lcd')

    # ---- 물리
    def step(self, dt):
        self.t += dt
        if self.nav:
            tx, ty, tyaw = self.nav
            d = math.hypot(tx - self.x, ty - self.y)
            sp = self.speed * dt
            if d <= sp:
                self.dist += d
                self.x, self.y, self.yaw = tx, ty, tyaw
                self.nav = None
                self.driver.on_nav_result(True)
            else:
                self.yaw = math.atan2(ty - self.y, tx - self.x)
                self.x += (tx - self.x) / d * sp
                self.y += (ty - self.y) / d * sp
                self.dist += sp
        else:
            self.yaw = wrap(self.yaw + self.w * dt)
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.dist += abs(self.v) * dt
        if self.guide_pending and self.t >= self.guide_pending[0]:
            _, gid, ok, detail = self.guide_pending
            self.guide_pending = None
            if gid == self.guide_id:
                self.log(f'guide 결과 {"OK" if ok else "못 찾음"} ({detail})', 'guide')
                self.driver.on_guide_result(ok, detail)
        if self.prec_pending and self.t >= self.prec_pending:
            self.prec_pending = None
            self.driver.on_precision_result(True, 'sim')


@dataclass
class Result:
    table: str
    plan: object
    ok: bool = False
    failed: str = ''
    sec: float = 0.0
    greet: list = field(default_factory=list)     # 인사한 노드
    checks: list = field(default_factory=list)    # 뒤돌아 확인한 노드
    arrive: list = field(default_factory=list)    # 도착 효과 노드
    reached: list = field(default_factory=list)
    end_node: str = ''
    yaw_err_deg: float = float('nan')             # 도착지에서 테이블 쪽과의 차이
    look_err_deg: list = field(default_factory=list)  # 확인할 때 로봇 앞 vs 왔던 노드(손님) 쪽 차이
    back_ok: bool = None                          # 복귀 결과 (안 했으면 None)
    blocked: list = field(default_factory=list)
    robot: object = None


def run_scenario(g, table, start='D1', spots=None, guest=None, lookahead=2, latency=0.1, back=True,
                 seat_sec=2.0, pause_at=None, obstacle=None, stops=None, check_turn=45.0, check_gap=0.8,
                 limit_sec=600.0, dt=0.05, check_every=False):
    """안내 한 번 (+ 복귀) -> Result.  pause_at = (노드, 초), obstacle = (노드, 초, 'DYNAMIC'|'STATIC')"""
    spots = load_guide_spots() if spots is None else spots
    guest = guest or Guest()
    plan = guide_plan(g, start, table, spots)
    sx, sy = g.xy(start)
    syaw = float(g.nodes[start].get('yaw', g.heading(start, plan.nodes[1]) if len(plan.nodes) > 1 else 0.0))
    robot = SimRobot(sx, sy, syaw, guest)
    drv = Driver(robot, stops or {}, reverse=ReverseControl(speed=0.08), policy=BlockPolicy(),
                 check_turn_deg=check_turn, check_gap=check_gap, check_every=check_every)
    drv.last_node = start
    drv.leave_from = start
    robot.driver = drv
    res = Result(table, plan, robot=robot)

    state = {'feeder': None, 'outbox': [], 'stage': 'guide', 'done_t': None, 'pause_until': None,
             'obst_until': None, 'paused_once': False, 'obst_once': False, 'ctl': []}
    # 서버 -> 로봇 Move · Control은 다음 주기에 (ROS처럼. 이벤트 콜백 안에서 바로 넣으면 Driver 처리 도중에 끼어듦)

    def send(specs):
        for m in specs:
            state['outbox'].append((robot.t + latency, m))

    def on_event(etype, cmd, node, reason):
        f = state['feeder']
        if f is None or cmd != f.cmd_id:
            return
        if etype == NODE_REACHED:
            send(f.on_reached(node))
            if state['stage'] == 'guide':
                if pause_at and node == pause_at[0] and not state['paused_once']:
                    state['paused_once'] = True
                    state['pause_until'] = robot.t + pause_at[1]
                    robot.log(f'[시험] PAUSE {pause_at[1]}초', 'ctl')
                    state['ctl'].append(PAUSE)
                if obstacle and node == obstacle[0] and not state['obst_once']:
                    state['obst_once'] = True
                    state['obst_until'] = robot.t + 0.4 + obstacle[1]
                    state['obst_from'] = robot.t + 0.4
        elif etype == CMD_DONE and node == f.plan.goal:
            if state['stage'] == 'guide':
                res.ok = True
                state['done_t'] = robot.t
            else:
                res.back_ok = True
            state['stage'] += '_done'
        elif etype == CMD_FAILED:
            if state['stage'] == 'guide':
                res.failed = reason
                res.end_node = node
                state['done_t'] = robot.t
            else:
                res.back_ok = False
            state['stage'] += '_done'
        elif etype == EV_BLOCKED:
            res.blocked.append(reason)

    robot.on_event = on_event
    state['feeder'] = MoveFeeder(plan, 100, lookahead)
    robot.log(f'안내 {table}: {plan.text()}  (도착지 {plan.goal}, {plan.face or "-"} 쪽 봄)', 'plan')
    send(state['feeder'].first())

    t = 0.0
    while t < limit_sec:
        # 서버 -> 로봇 (지연)
        due = [m for at, m in state['outbox'] if at <= robot.t]
        state['outbox'] = [(at, m) for at, m in state['outbox'] if at > robot.t]
        for m in due:
            robot.log(m.text(), 'move')
            drv.on_move(m.cmd_id, m.append, m.drive, m.waypoints, m.guide, m.goal)
        for c in state['ctl']:
            drv.on_control(c)
        state['ctl'] = []
        if state['pause_until'] and robot.t >= state['pause_until']:
            state['pause_until'] = None
            robot.log('[시험] RESUME', 'ctl')
            drv.on_control(RESUME)
        if state['obst_until'] is not None:
            on = state['obst_from'] <= robot.t < state['obst_until']
            drv.set_obstacle((obstacle[2] if len(obstacle) > 2 else 'DYNAMIC') if on else '')
            if robot.t >= state['obst_until']:
                state['obst_until'] = None
                drv.set_obstacle('')
        # 안내 끝 -> (착석 시간) -> 복귀
        if state['stage'] == 'guide_done':
            if not back:
                break
            if robot.t >= state['done_t'] + (seat_sec if res.ok else 0.5):
                here = res.plan.goal if res.ok else (drv.last_node or start)
                bp = return_plan(g, here, start)
                state['feeder'] = MoveFeeder(bp, 200, lookahead)
                state['stage'] = 'back'
                robot.log(f'복귀: {bp.text()}', 'plan')
                send(state['feeder'].first())
        if state['stage'] == 'back_done':
            break
        drv.tick(robot.t, (robot.x, robot.y, robot.yaw))
        robot.step(dt)
        t += dt

    gt = [x for x in robot.guides]
    res.greet = [n for _, a, n, *_ in gt if a == 'greet']
    res.checks = [n for _, a, n, *_ in gt if a == 'watch']
    res.arrive = [n for _, a, n, *_ in gt if a == 'arrive']
    guide_cmd_events = [e for e in robot.events]
    res.reached = [n for _, et, n, _ in guide_cmd_events if et == NODE_REACHED]
    for _, a, n, x, y, yaw in gt:
        if a == 'watch' and n in plan.nodes and plan.nodes.index(n) > 0:
            prev = plan.nodes[plan.nodes.index(n) - 1]
            px, py = g.xy(prev)
            res.look_err_deg.append(abs(math.degrees(wrap(math.atan2(py - y, px - x) - yaw))))
    res.sec = state['done_t'] if state['done_t'] is not None else robot.t
    if res.ok:
        res.end_node = plan.goal
        arr = [x for x in gt if x[1] == 'arrive']
        if arr and plan.face:
            _, _, _, x, y, yaw = arr[0]
            res.yaw_err_deg = abs(math.degrees(wrap(g.heading(plan.goal, plan.face) - yaw)))
    elif not res.end_node:
        res.end_node = drv.last_node
    return res


def print_log(res):
    for t, kind, text in res.robot.lines:
        mark = {'event': '  ', 'guide': '★ ', 'move': '→ ', 'plan': '# ', 'ctl': '! ', 'lcd': '! ', 'prec': '  '}.get(kind, '  ')
        print(f'{t:7.2f}s {mark}{text}')


def summary_row(r):
    out = 'OK' if r.ok else ('실패 ' + r.failed if r.failed else '시간 초과')
    back = {None: '-', True: 'OK', False: '실패'}[r.back_ok]
    yaw = '' if math.isnan(r.yaw_err_deg) else f'{r.yaw_err_deg:.0f}'
    look = ','.join(f'{d:.0f}' for d in r.look_err_deg) or '-'
    return (f'{r.table:4s} {r.plan.goal:4s} {" ".join(r.plan.nodes):30s} {",".join(r.greet) or "-":5s} '
            f'{",".join(r.checks) or "-":9s} {look:9s} {",".join(r.arrive) or "-":5s} {yaw:>4s} {r.sec:6.1f} '
            f'{back:4s} {out}')


def main(argv=None):
    if argv is None:
        try:
            from rclpy.utilities import remove_ros_args
            argv = remove_ros_args(sys.argv)[1:]
        except Exception:  # noqa: BLE001
            argv = sys.argv[1:]
    ap = argparse.ArgumentParser(description='안내 시뮬레이터 (PC, ROS 없이 core.Driver를 node_graph 위에서)')
    ap.add_argument('tables', nargs='*', help='안내할 테이블 (기본: T1~T6 전부)')
    ap.add_argument('--start', default='D1', help='출발 도킹 (기본 D1)')
    ap.add_argument('--lookahead', type=int, default=2, help='서버가 앞서 보내는 노드 수 (0 = 한 번에 전부)')
    ap.add_argument('--latency', type=float, default=0.1, help='서버 APPEND 지연 (초)')
    ap.add_argument('--lost-at', default='', help='이 노드(들)의 확인에서 손님 못 찾음. 예: N3 또는 N1,N3')
    ap.add_argument('--greet-fail', action='store_true', help='입구 인사에서 손님이 안 옴')
    ap.add_argument('--react', type=float, default=1.0, help='손님을 찾는 데 걸리는 초')
    ap.add_argument('--wait', type=float, default=30.0, help='못 찾을 때 포기까지 초 (guide wait_sec)')
    ap.add_argument('--no-return', action='store_true', help='안내 끝나고 복귀 안 함')
    ap.add_argument('--pause-at', default='', help='노드:초  이 노드 도착 뒤 PAUSE, 초 뒤 RESUME')
    ap.add_argument('--obstacle', default='', help='노드:초[:STATIC]  이 노드를 지나 초 동안 막힘')
    ap.add_argument('--check-turn', type=float, default=45.0, help='guide_check_turn_deg')
    ap.add_argument('--check-gap', type=float, default=0.8, help='guide_check_gap_m')
    ap.add_argument('--every-node', action='store_true', help='guide_check_every_node: 지나가는 노드마다 확인')
    ap.add_argument('--stops', default='', help='precision_stops.yaml (복귀 도킹 정밀 정차도 흉내)')
    ap.add_argument('--graph', default='', help='node_graph.yaml')
    ap.add_argument('--server-config', default='', help='main_service.yaml (guide_spots)')
    ap.add_argument('-v', '--verbose', action='store_true', help='시간순 로그')
    a = ap.parse_args(argv)

    g = GuideMap(a.graph)
    spots = load_guide_spots(a.server_config)
    tables = a.tables or sorted(spots)
    guest = Guest(set(filter(None, a.lost_at.split(','))), a.greet_fail, a.react, a.wait)
    pause = None
    if a.pause_at:
        n, s = a.pause_at.split(':')
        pause = (n, float(s))
    obst = None
    if a.obstacle:
        parts = a.obstacle.split(':')
        obst = (parts[0], float(parts[1]), parts[2].upper() if len(parts) > 2 else 'DYNAMIC')
    stops = load_precision_stops(a.stops) if a.stops else {}

    print(f'맵 {g.file}\n안내 자리 {spots}\n')
    rows = []
    for t in tables:
        r = run_scenario(g, t, a.start, spots, guest, a.lookahead, a.latency, not a.no_return,
                         pause_at=pause, obstacle=obst, stops=stops, check_turn=a.check_turn, check_gap=a.check_gap,
                         check_every=a.every_node)
        rows.append(r)
        if a.verbose:
            print(f'==== {t} ====')
            print_log(r)
            print()
    print(f'{"테이블":4s} {"도착":4s} {"경로":30s} {"인사":5s} {"확인노드":9s} {"확인방향°":9s} {"도착효과":5s} '
          f'{"yaw°":>4s} {"초":>6s} {"복귀":4s} 결과')
    print('  확인방향° = 뒤돌아 섰을 때 로봇 앞과 손님(왔던 노드) 쪽의 차이. 카메라 화각 반쯤(약 30°) 안이어야 손님이 보임')
    print('  yaw° = 도착지에서 테이블 쪽과의 차이')
    for r in rows:
        print(summary_row(r))
    for r in rows:
        if r.blocked:
            print(f'{r.table}: BLOCKED {r.blocked}')
    bad = [r for r in rows if not r.ok and not (guest.lost_at or guest.greet_fail)]
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
