"""fleet_agent 핵심 로직 (ROS를 모른다. agent_node가 ROS 입출력을 이어 준다)

- StepQueue      : 서버 Move를 waypoint 하나씩(Step)으로 펼쳐 둔 줄
- ReverseControl : cmd_vel 직선 후진 (위치 · 방향 P제어). command_axis = 정해진 축(heading) 그대로 후진
- BlockPolicy    : 장애물로 멈췄을 때 언제 서버에 BLOCKED를 보낼지 (사람 / 물건)
- ObstacleClass  : LiDAR 거리 변화로 사람(DYNAMIC) / 물건(STATIC) 구분
- Driver         : 노드 단위 주행 상태기계 (fleet_msgs README 5장 로봇 규칙)
                   + 안내 (Move.guide: 출발 전 인사 · 꺾이는 노드에서 뒤돌아 확인 · 도착 효과, 손님 확인은 Guide 노드)
"""
import math
from dataclasses import dataclass

import yaml

FORWARD = 0
REVERSE = 1

# RobotState.state
IDLE, MOVING, PAUSED, BLOCKED, ERROR = 0, 1, 2, 3, 4
# RobotEvent.type
NODE_REACHED, CMD_DONE, CMD_FAILED, EV_BLOCKED = 0, 1, 2, 4
# Control.command
PAUSE, RESUME, CANCEL = 0, 1, 2


def wrap(a):
    """각도를 -pi ~ pi 로"""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def load_precision_stops(path):
    """fleet_msgs/docs/precision_stops.yaml -> {node_id: {method, marker, markers, back, ...}}"""
    if not path:
        return {}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for node, cfg in data.items():
        if isinstance(cfg, dict) and cfg.get('method') in ('dock', 'align', 'tape'):
            out[str(node)] = dict(cfg)
    return out


@dataclass
class Step:
    cmd_id: int
    drive: int
    node_id: str
    x: float
    y: float
    yaw: float
    guide: bool = False   # Move.guide (안내 중)
    goal: str = ''        # Move.goal_node (최종 도착지, '' = 모름)


class StepQueue:
    """REPLACE = 비우고 새 cmd_id로, APPEND = 지금 cmd_id와 같을 때만 뒤에 이어 붙인다"""

    def __init__(self):
        self.steps = []
        self.cmd_id = 0

    def apply(self, cmd_id, append, drive, waypoints, guide=False, goal=''):
        if append:
            if cmd_id != self.cmd_id:
                return False  # 다른 명령에 붙이려는 APPEND는 버린다
        else:
            self.steps.clear()
            self.cmd_id = cmd_id
        for w in waypoints:
            self.steps.append(Step(cmd_id, drive, w[0], w[1], w[2], w[3], bool(guide), goal or ''))
        return True

    def clear(self):
        self.steps.clear()

    def __len__(self):
        return len(self.steps)


class ReverseControl:
    """뒤가 진행 방향이 되게 (tx, ty)까지 직선 후진"""

    def __init__(self, speed=0.06, k_ang=1.8, max_w=0.8, tol=0.03, turn_first=0.35):
        self.speed = speed
        self.k_ang = k_ang
        self.max_w = max_w
        self.tol = tol
        self.turn_first = turn_first  # 방향 오차가 이보다 크면 먼저 제자리 회전

    def command(self, pose, target, start=None):
        """-> (v, w, done). start를 주면 목표를 지나쳤는지도 본다"""
        x, y, yaw = pose
        tx, ty = target
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < self.tol:
            return 0.0, 0.0, True
        if start is not None:  # 출발점 -> 목표 방향으로 목표를 지나갔으면 도착
            sx, sy = start
            if (tx - sx) * (x - tx) + (ty - sy) * (y - ty) > 0.0:
                return 0.0, 0.0, True
        face = wrap(math.atan2(dy, dx) + math.pi)  # 로봇 앞이 볼 방향 (뒤로 가니까 반대)
        err = wrap(face - yaw)
        w = clamp(self.k_ang * err, self.max_w)
        if abs(err) > self.turn_first:
            return 0.0, w, False
        v = -min(self.speed, max(0.02, d)) * math.cos(err)
        return v, w, False

    def command_axis(self, pose, target, axis, heading_err=None):
        """축 고정 후진: 로봇 앞 방향을 축으로 유지하고, 축 위로 잰 남은 거리만 본다 -> (v, w, done)
        (마커로 맞춘 축을 Nav2 좌표 쪽으로 다시 꺾지 않게. 좌우 위치는 축이 정한다)
        heading_err: 방향 오차를 odom으로 잰 값 (AMCL이 고칠 때마다 꺾이지 않게). 없으면 맵 yaw로"""
        x, y, yaw = pose
        b = axis + math.pi                      # 뒤로 가는 방향
        remain = (target[0] - x) * math.cos(b) + (target[1] - y) * math.sin(b)
        if remain < self.tol:
            return 0.0, 0.0, True
        err = heading_err if heading_err is not None else wrap(axis - yaw)
        w = clamp(self.k_ang * err, self.max_w)
        return -min(self.speed, max(0.02, remain)), w, False


def turn_command(yaw, target, k=1.8, max_w=0.8, tol=0.08):
    """제자리 회전 -> (w, done)"""
    err = wrap(target - yaw)
    if abs(err) < tol:
        return 0.0, True
    w = clamp(k * err, max_w)
    if abs(w) < 0.15:
        w = math.copysign(0.15, w)  # 너무 느리면 안 돈다
    return w, False


class ObstacleClass:
    """막힌 동안 앞 거리 변화로 종류를 가른다 (로봇은 멈춰 있다고 본다)
    - 거리가 window 동안 move_thresh 넘게 흔들리면 DYNAMIC (사람)
    - static_sec 동안 그대로면 STATIC (물건)
    - 그 전에는 UNKNOWN
    - person=True (카메라 YOLO)면 바로 DYNAMIC
    """

    def __init__(self, window=1.5, move_thresh=0.04, static_sec=2.0, settle_sec=0.5):
        self.window = window
        self.move_thresh = move_thresh
        self.static_sec = static_sec
        self.settle_sec = settle_sec  # 막힌 직후 로봇이 서는 동안은 거리 변화를 안 본다
        self.since = None
        self.hist = []
        self.dynamic_seen = False

    def reset(self):
        self.since = None
        self.hist = []
        self.dynamic_seen = False

    def update(self, t, dist, person=False):
        """dist = 앞 장애물 거리 (None = 막힘 없음) -> kind 또는 '' (막힘 없음)"""
        if dist is None:
            self.reset()
            return ''
        if self.since is None:
            self.since = t
        if t - self.since >= self.settle_sec:
            self.hist.append((t, dist))
            self.hist = [(ht, hd) for ht, hd in self.hist if t - ht <= self.window]
        if person:
            self.dynamic_seen = True
        if len(self.hist) >= 2:
            ds = [hd for _, hd in self.hist]
            if max(ds) - min(ds) > self.move_thresh:
                self.dynamic_seen = True
        if self.dynamic_seen:
            return 'DYNAMIC'
        if t - self.since >= self.static_sec:
            return 'STATIC'
        return 'UNKNOWN'


class BlockPolicy:
    """멈춘 뒤 서버에 BLOCKED를 보낼 때 (README 장애물 단계)
    - 사람(DYNAMIC): dynamic_wait 초 기다림 -> dynamic_ask 초 동안 LCD '지나갈게요' -> 보고
    - 물건(STATIC) : static_wait 초 확인 -> 보고
    - 모름(UNKNOWN): unknown_wait 초 뒤 보고
    보고는 한 번만. 길이 비면 처음부터.
    """

    def __init__(self, dynamic_wait=3.0, dynamic_ask=5.0, static_wait=2.0, unknown_wait=8.0):
        self.dynamic_wait = dynamic_wait
        self.dynamic_ask = dynamic_ask
        self.static_wait = static_wait
        self.unknown_wait = unknown_wait
        self.since = None
        self.reported = False

    def update(self, t, kind):
        """kind '' = 막힘 없음 -> (ask, report_kind 또는 None)"""
        if not kind:
            self.since = None
            self.reported = False
            return False, None
        if self.since is None:
            self.since = t
        el = t - self.since
        ask = False
        due = None
        if kind == 'DYNAMIC':
            ask = self.dynamic_wait <= el < self.dynamic_wait + self.dynamic_ask
            due = self.dynamic_wait + self.dynamic_ask
        elif kind == 'STATIC':
            due = self.static_wait
        else:
            due = self.unknown_wait
        if not self.reported and el >= due:
            self.reported = True
            return ask, kind
        return ask, None


class Driver:
    """노드 단위 주행 상태기계

    io (agent_node가 넘겨줌)
      nav_go(x, y, yaw)        Nav2 목표 보내기 (결과는 on_nav_result)
      nav_cancel()
      cmd_vel(v, w)
      event(type, cmd_id, node, reason)
      precision(req)           Docking 노드에 요청 (결과는 on_precision_result)
      precision_cancel()
      direction(text)          Obstacle 노드에 지금 볼 방향 (FORWARD / REVERSE / STOP)
      lcd_override(text)       장애물 안내 문구 (None = 서버 문구로 돌아감)
      guide(req)               Guide 노드에 요청 {action: greet / watch / arrive} (결과는 on_guide_result)
      guide_cancel()
      log(text)

    규칙 (fleet_msgs README 5장)
      - 받은 노드까지만 간다. 노드마다 NODE_REACHED
      - 같은 방향 APPEND는 멈추지 않고 이어서, drive가 바뀌면 그 노드에서 멈추고 제자리 회전 후 출발
      - FORWARD = Nav2, REVERSE = cmd_vel 직선 후진
      - 중간 노드는 위치만 통과, 줄 끝 노드에서만 yaw를 맞춘다
      - 도착지(Move.goal_node)가 dock / tape 정밀 정차면 끝낸 뒤 CMD_DONE. align 노드는 지날 때마다 맞추고 이어 간다
        줄 끝이 도착지가 아니면(길이 막혀 서버가 거기까지만 보냄) 정밀 정차 없이 CMD_DONE하고 다음 Move를 기다린다
        goal_node가 비어 있으면(예전 서버) 줄 끝을 도착지로 본다
      - leave (precision_stops의 노드.leave): 그 노드에서 CMD_DONE 한 뒤(또는 켤 때 서 있던 노드) 다음 출발 전에
        docking에 맡긴다. 예) D4 leave=undock: 마커로 N15까지 나와 돌고 정렬 -> to 노드(N15) 도착으로 침
                              P1 leave=tape_exit: 테이프 선까지 앞으로 빼고 -> 그다음 Nav2
        hold_axis: 맞춘 방향을 축으로 이어지는 후진을 그 축 그대로 (출발 회전 · 끝 노드 yaw 회전 생략)
                   방향은 odom으로 유지 (io.odom_yaw가 있으면). 맵(AMCL) 보정에 따라 꺾이지 않는다
      - 축 고정 후진으로 tape 노드(P1)에 갈 때는 좌표까지 가지 않고 tape에 바로 맡긴다 (tape_approach):
        tape가 바닥 가로선을 보며 후진해 멈추고 삽입까지 -> NODE_REACHED · CMD_DONE (좌표가 조금 틀려도 됨)
    안내 (Move.guide = true): 서버는 "안내 중"과 도착지만 알려 주고, 나머지는 로봇이 판단한다
      - 인사 : 안내가 아닌 상태에서 안내 Move를 처음 받으면, 떠나기 전에 Guide 'greet' (앞 카메라로 손님 기다림)
      - 확인 : 꺾이는 노드(들어온 방향과 나갈 방향이 check_turn 넘게 다름)에서 멈춤 -> 뒤로 돎 -> Guide 'watch'
               check_every=True면 꺾임 · check_gap과 상관없이 지나가는 노드마다 (도착지는 빼고)
               -> 다음 노드 쪽으로 돎 -> 이어 감. 바로 전 인사 · 확인 자리에서 check_gap m 안이면 건너뜀
      - 도착 : 도착지(goal_node)에서 CMD_DONE 뒤 Guide 'arrive' (도착 효과, 기다리지 않음). 안내는 정밀 정차 안 함
      - 손님을 못 찾으면 그 자리에 멈추고 CMD_FAILED reason="guest_lost:<노드>:<이유>" (고장 아님, 줄 비움)
    """

    def __init__(self, io, stops, pass_tol=0.08, yaw_tol=0.15, done_grace=0.5, reverse=None, policy=None,
                 nav_retries=1, check_turn_deg=45.0, check_gap=0.8, check_every=False):
        self.io = io
        self.stops = stops
        self.pass_tol = pass_tol      # 중간 노드 통과 판정 거리 (다음 노드로 바로 넘어감)
        self.yaw_tol = yaw_tol
        self.done_grace = done_grace  # 줄이 비고 이만큼 기다렸다가 끝 처리 (연달아 오는 APPEND를 기다림)
        self.rev = reverse or ReverseControl()
        self.policy = policy or BlockPolicy()
        self.queue = StepQueue()
        self.cur = None
        self.phase = 'idle'           # idle / turn / nav / reverse / align / grace / final_turn / precision
        #                               guide_greet / guide_turn / guide_watch (안내)
        self.after_turn = ''          # turn 끝나고: move / finish
        self.turn_target = 0.0
        self.seg_start = None
        self.last_drive = None        # 마지막으로 달린 방향 (방향 바뀜 판단)
        self.last_node = ''
        self.paused = False
        self.error = False
        self.nav_state = ''           # '' / running / ok / fail
        self.prec_state = ''          # '' / running / ok
        self.grace_until = 0.0
        self.obstacle = ''            # 지금 진행 방향 장애물 종류 ('' = 없음)
        self.holding = False          # 장애물로 멈춘 중
        self.now = 0.0
        self.nav_retries = nav_retries  # Nav2 실패 때 다시 보내는 횟수 (그래도 안 되면 CMD_FAILED)
        self.nav_tries = 0
        self.prec_detail = ''         # 정밀 정차 실패 이유 -> CMD_DONE.reason
        self.guide_state = ''         # '' / running / ok / fail
        self.guide_detail = ''
        self.guide_after = None       # guide 단계가 끝나면 할 일 (함수)
        self.in_guide = False         # 안내 중 (인사를 이미 함)
        self.check_turn = math.radians(check_turn_deg)
        self.check_gap = check_gap
        self.check_every = check_every  # True = 꺾임과 상관없이 지나가는 노드마다 확인 (도착지는 빼고)
        self.check_xy = None          # 마지막으로 인사 · 확인한 자리
        self.prev_xy = None           # 바로 전에 도착한 노드 (꺾임 판단)
        self.came_xy = None           # 지금 선 노드 직전 노드 (손님이 따라오는 쪽. 다음 노드를 늦게 받을 때 꺾임 판단)
        self.leave_from = ''          # 여기서 출발할 때 leave 정밀 동작을 먼저 (precision_stops의 leave)
        self.leave_req = None
        self.prec_ok = True
        self.axis = None              # 축 고정 후진 방향 (leave hold_axis로 정함, FORWARD 출발 때 해제)
        self.axis_odom = None         # 같은 축을 odom yaw로 (방향 유지는 이것으로)
        self.prec_done_node = ''      # tape_approach로 정밀 정차를 이미 끝낸 노드
        self.quiet_obstacle = False   # leave quiet_to_next: 다음 노드 도착까지 LiDAR 장애물 판단 안 함 (좁은 틈 통과)
        self.skip_turn = False        # 다음 방향 바뀜 회전 생략 (축을 이미 맞춤)

    # ------------------------------------------------------------ 입력
    def on_move(self, cmd_id, append, drive, waypoints, guide=False, goal=''):
        if not waypoints:
            return
        if not guide:
            self.in_guide = False     # 안내가 끝났다 (복귀 · 서빙)
        if not append:  # REPLACE: 지금 가던 것까지 버리고 새로
            if self.phase == 'leave' and self.leave_req:
                self.leave_from = self.leave_req['node']   # 아직 못 떠났다 -> 새 명령 출발 때 다시
            self._stop_motion()
            self.queue.apply(cmd_id, False, drive, waypoints, guide, goal)
            self.cur = None
            self.phase = 'idle'
            self.error = False
            return
        if not self.queue.apply(cmd_id, True, drive, waypoints, guide, goal):
            self.io.log(f'다른 명령({cmd_id})에 붙이는 APPEND 무시 (지금 {self.queue.cmd_id})')
            return
        if self.phase == 'precision':
            # 끝 노드인 줄 알고 정밀 정차를 시작했는데 길이 더 왔다 = 지나가는 노드였다 -> 그만두고 이어 감
            self.io.log(f'{self.cur.node_id}: 길이 더 와서 정밀 정차 취소')
            self.io.precision_cancel()
            self.prec_state = ''
            self.phase = 'grace'
            self.grace_until = self.now

    def on_control(self, cmd):
        if cmd == PAUSE:
            self.paused = True
            self._stop_motion(keep=True)
        elif cmd == RESUME:
            self.paused = False
            self.error = False
            self._resume()
        elif cmd == CANCEL:
            self._stop_motion()
            self.in_guide = False
            self.queue.clear()
            self.cur = None
            self.phase = 'idle'
            self.paused = False

    def on_nav_result(self, ok):
        if self.phase == 'nav' and self.nav_state == 'running':
            self.nav_state = 'ok' if ok else 'fail'

    def on_precision_result(self, ok, detail=''):
        if self.phase in ('align', 'precision', 'leave', 'tape_approach') and self.prec_state == 'running':
            self.prec_state = 'ok'
            self.prec_ok = ok
            if not ok:
                self.io.log(f'정밀 정차 실패, Nav2 위치로 진행: {detail}')
                self.prec_detail = f'{self.cur.node_id if self.cur else ""}:{detail}'

    def on_guide_result(self, ok, detail=''):
        if self.phase in ('guide_greet', 'guide_watch') and self.guide_state == 'running':
            self.guide_state = 'ok' if ok else 'fail'
            self.guide_detail = detail

    def set_obstacle(self, kind):
        self.obstacle = kind or ''

    # ------------------------------------------------------------ 상태
    def run_state(self):
        if self.error:
            return ERROR
        if self.paused:
            return PAUSED
        if self.holding:
            return BLOCKED
        if self.phase == 'idle':
            return IDLE
        return MOVING

    # ------------------------------------------------------------ 주기 처리
    def tick(self, now, pose):
        self.now = now
        if pose is None or self.paused or self.error:
            return
        moving = self.phase in ('nav', 'reverse') and not self.quiet_obstacle   # 정밀 동작(tape 등) 중에는 장애물 판단 안 함
        if moving or self.holding:
            ask, report = self.policy.update(now, self.obstacle)
            if self.obstacle:
                if not self.holding:
                    self.holding = True
                    self._stop_motion(keep=True, keep_dir=True)  # 방향은 그대로 둬야 Obstacle 노드가 계속 본다
                    self.io.log(f'장애물({self.obstacle}) -> 정지')
                self.io.lcd_override('지나갈게요' if ask else None)
                if report:
                    nxt = self.cur.node_id if self.cur else ''
                    self.io.event(EV_BLOCKED, self.queue.cmd_id, nxt, f'{report}:{self.last_node}->{nxt}')
                return
            if self.holding:  # 길이 비었다
                self.holding = False
                self.io.lcd_override(None)
                self.io.log('장애물 없어짐 -> 다시 출발')
                self._resume()
                return

        if self.phase == 'idle':
            if len(self.queue) and not self._late_check(pose):
                self._start_next(pose)
        elif self.phase in ('turn', 'final_turn', 'guide_turn'):
            w, done = turn_command(pose[2], self.turn_target, tol=self.yaw_tol)
            self.io.cmd_vel(0.0, w)
            if done:
                self.io.cmd_vel(0.0, 0.0)
                if self.phase == 'turn':
                    self._begin_move(pose)
                elif self.phase == 'guide_turn':
                    self.guide_after(pose)
                else:
                    self._precision_or_done()
        elif self.phase in ('guide_greet', 'guide_watch'):
            if self.guide_state == 'ok':
                self.guide_state = ''
                self.guide_after(pose)
            elif self.guide_state == 'fail':
                self._guest_lost()
        elif self.phase == 'nav':
            self._tick_nav(pose)
        elif self.phase == 'reverse':
            if self.axis is not None:
                oy = self._odom_yaw()
                err = wrap(self.axis_odom - oy) if (oy is not None and self.axis_odom is not None) else None
                v, w, done = self.rev.command_axis(pose, (self.cur.x, self.cur.y), self.axis, err)
            else:
                v, w, done = self.rev.command(pose, (self.cur.x, self.cur.y), self.seg_start)
            if done:
                nxt = self.queue.steps[0] if len(self.queue) else None
                if not (nxt and nxt.drive == REVERSE):
                    self.io.cmd_vel(0.0, 0.0)
                self._reached(pose)
            else:
                self.io.cmd_vel(v, w)
        elif self.phase == 'align' or self.phase == 'precision':
            if self.prec_state == 'ok':
                self.prec_state = ''
                if self.phase == 'align':
                    self._after_node(pose)
                else:
                    self._done()
        elif self.phase == 'leave':
            if self.prec_state == 'ok':
                self.prec_state = ''
                req, self.leave_req = self.leave_req or {}, None
                if self.prec_ok and req.get('quiet_to_next'):
                    self.quiet_obstacle = True
                if self.prec_ok and req.get('hold_axis'):
                    self.axis = pose[2]
                    self.axis_odom = self._odom_yaw()
                    self.skip_turn = True
                if self.prec_ok and req.get('to') == self.cur.node_id:
                    self.io.log(f'{req.get("node")} {req.get("method")} -> {self.cur.node_id} 도착')
                    self._reached(pose)
                else:
                    self._depart(pose)
        elif self.phase == 'tape_approach':
            if self.prec_state == 'ok':
                self.prec_state = ''
                self.prec_done_node = self.cur.node_id
                self._reached(pose)
        elif self.phase == 'grace':
            if len(self.queue):
                if not self._late_check(pose):
                    self._start_next(pose)
            elif now >= self.grace_until:
                self._finish(pose)

    # ------------------------------------------------------------ 내부
    def _tick_nav(self, pose):
        d = math.hypot(self.cur.x - pose[0], self.cur.y - pose[1])
        nxt = self.queue.steps[0] if len(self.queue) else None
        if nxt and nxt.drive == FORWARD and d < self.pass_tol and not self._is_check(self.cur, nxt, self.prev_xy):
            self._reached(pose)  # 중간 노드: 위치만 통과, 다음 목표로 바로 (손님 확인 노드는 끝까지 가서 선다)
            return
        if self.nav_state == 'ok':
            self._reached(pose)
        elif self.nav_state == 'fail':
            if self.nav_tries < self.nav_retries:  # 한 번 더 보내 본다
                self.nav_tries += 1
                self.io.log(f'Nav2 실패: {self.cur.node_id} -> 다시 시도 ({self.nav_tries}/{self.nav_retries})')
                self.nav_state = 'running'
                self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
                return
            self.io.log(f'Nav2 실패: {self.cur.node_id}')
            self.io.event(CMD_FAILED, self.queue.cmd_id, self.cur.node_id, 'nav2_failed')
            self.error = True
            self.phase = 'idle'
            self.io.direction('STOP')

    def _start_next(self, pose):
        self.cur = self.queue.steps.pop(0)
        if self.prev_xy is None:
            self.prev_xy = (pose[0], pose[1])
        if self.cur.guide and not self.in_guide:
            # 안내 첫 출발: 떠나기 전에 앞(카메라)으로 손님을 기다린다
            self.in_guide = True
            self.check_xy = (pose[0], pose[1])
            self._guide_request('greet', self.last_node or self.cur.node_id, self._depart)
            return
        self._depart(pose)

    def _depart(self, pose):
        leave = self.stops.get(self.leave_from, {}).get('leave') if self.leave_from else None
        if leave:  # 이 노드를 떠날 때 할 정밀 동작 (언도킹 · 테이프 빼기)
            self.leave_req = dict(leave, node=self.leave_from)
            self.leave_from = ''
            self.phase = 'leave'
            self.prec_state = 'running'
            self.io.direction('STOP')
            self.io.precision(self.leave_req)
            return
        if self.last_drive is not None and self.cur.drive != self.last_drive and self.skip_turn:
            self.skip_turn = False    # 마커로 축을 이미 맞춤 -> 돌지 않고 바로
            self._begin_move(pose)
            return
        if self.last_drive is not None and self.cur.drive != self.last_drive:
            # 방향이 바뀜: 이 자리(lock 노드)에서 멈추고 돈다
            heading = math.atan2(self.cur.y - pose[1], self.cur.x - pose[0])
            self.turn_target = wrap(heading + math.pi) if self.cur.drive == REVERSE else heading
            self.phase = 'turn'
            self.io.direction('STOP')
            return
        self._begin_move(pose)

    def _begin_move(self, pose):
        self.seg_start = (pose[0], pose[1])
        if self.cur.drive == FORWARD:
            self.axis = None
            self.axis_odom = None
            self.skip_turn = False
            self.phase = 'nav'  # nav_go 결과가 바로 와도 받을 수 있게 먼저
            self.nav_state = 'running'
            self.nav_tries = 0
            self.io.direction('FORWARD')
            self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
        else:
            stop = self.stops.get(self.cur.node_id, {})
            if (self.axis is not None and stop.get('method') == 'tape' and stop.get('approach', True)
                    and self._precise_here(self.cur)):
                # 축 고정 후진으로 테이프 노드: 좌표 대신 바닥 가로선으로 멈추게 tape에 바로 맡김
                self.io.cmd_vel(0.0, 0.0)  # 여기서부터 /cmd_vel은 docking(tape)이 씀
                self.phase = 'tape_approach'
                self.prec_state = 'running'
                self.io.direction('STOP')   # 픽업대 · 벽에 붙어야 하니 LiDAR 장애물 판단 끔
                self.io.precision(dict(stop, node=self.cur.node_id, approach=True))
                return
            self.phase = 'reverse'
            self.io.direction('REVERSE')

    def _resume(self):
        if self.cur is None:
            return
        if self.phase == 'nav':
            self.nav_state = 'running'
            self.io.nav_go(self.cur.x, self.cur.y, self.cur.yaw)
        elif self.phase == 'precision' or self.phase == 'align':
            self._request_precision(self.phase == 'align')
        elif self.phase == 'tape_approach':
            self.prec_state = 'running'
            self.io.precision(dict(self.stops.get(self.cur.node_id, {}), node=self.cur.node_id, approach=True))
        elif self.phase == 'leave' and self.leave_req:
            self.prec_state = 'running'
            self.io.precision(self.leave_req)
        elif self.phase in ('guide_greet', 'guide_watch'):
            self.guide_state = 'running'
            self.io.guide({'action': 'greet' if self.phase == 'guide_greet' else 'watch',
                           'node': self.cur.node_id if self.cur else self.last_node})

    def _reached(self, pose):
        step = self.cur
        self.quiet_obstacle = False
        self.last_node = step.node_id
        self.last_drive = step.drive
        self.nav_state = ''
        self.io.event(NODE_REACHED, step.cmd_id, step.node_id, '')
        nxt = self.queue.steps[0] if len(self.queue) else None
        came = self.prev_xy
        check = self._is_check(step, nxt, came)
        self.prev_xy = (step.x, step.y)
        self.came_xy = came
        if check:
            self._begin_check(pose, came)
            return
        stop = self.stops.get(step.node_id)
        if stop and stop['method'] == 'align':
            self.io.cmd_vel(0.0, 0.0)
            self.io.nav_cancel()
            self.phase = 'align'
            self.io.direction('STOP')
            self._request_precision(True)
            return
        self._after_node(pose)

    def _after_node(self, pose):
        if len(self.queue):
            self._start_next(pose)
            return
        self.phase = 'grace'
        self.grace_until = self.now + self.done_grace

    def _finish(self, pose):
        """줄 끝 노드: yaw 맞추기 -> (dock / tape) -> CMD_DONE"""
        self.io.direction('STOP')
        stop = self.stops.get(self.cur.node_id, {})
        keep_axis = self.axis is not None or stop.get('no_final_turn')   # 축 그대로 (테이프가 이어서 맞춤)
        if self.cur.drive == REVERSE and not keep_axis and abs(wrap(self.cur.yaw - pose[2])) > self.yaw_tol:
            self.turn_target = self.cur.yaw  # FORWARD는 Nav2가 이미 맞춤
            self.phase = 'final_turn'
            return
        self._precision_or_done()

    def _precision_or_done(self):
        if self.prec_done_node and self.prec_done_node == self.cur.node_id:
            self.prec_done_node = ''      # tape_approach에서 이미 함
            self._done()
            return
        stop = self.stops.get(self.cur.node_id)
        if stop and stop['method'] in ('dock', 'tape') and self._precise_here(self.cur):
            self.phase = 'precision'
            self._request_precision(False)
            return
        self._done()

    def _at_goal(self, step):
        """이 노드가 이번 이동의 도착지인가 (Move.goal_node). 모르면(예전 서버) 줄 끝(뒤에 남은 노드 없음)이면 도착지"""
        if step.goal:
            return step.node_id == step.goal
        return len(self.queue) == 0

    def _precise_here(self, step):
        """정밀 정차(dock · tape)를 할 곳인가: 도착지이고 안내가 아닐 때 (안내는 Nav2 위치 · yaw로만 섬)"""
        return self._at_goal(step) and not step.guide

    def _request_precision(self, align):
        stop = dict(self.stops.get(self.cur.node_id, {}))
        stop['node'] = self.cur.node_id
        self.prec_state = 'running'
        self.io.precision(stop)

    def _done(self):
        # 정밀 정차를 못 했으면 이유를 reason에 (서버가 경고만 띄우고 작업은 이어 감)
        self.io.event(CMD_DONE, self.queue.cmd_id, self.cur.node_id, self.prec_detail)
        self.prec_detail = ''
        if self.stops.get(self.cur.node_id, {}).get('leave') and self._precise_here(self.cur):
            self.leave_from = self.cur.node_id   # 다음 출발 때 leave 먼저 (정밀 정차한 도착지에서만)
        self.phase = 'idle'
        self.io.direction('STOP')
        if self.cur.guide and self._at_goal(self.cur):
            self.io.guide({'action': 'arrive', 'node': self.cur.node_id})  # 도착 효과 (기다리지 않음)

    def _odom_yaw(self):
        f = getattr(self.io, 'odom_yaw', None)
        try:
            return f() if f else None
        except Exception:
            return None

    # ------------------------------------------------------------ 안내
    def _is_check(self, step, nxt, came):
        """안내 중 꺾이는 노드인가 (came = 직전 노드에서 들어온 방향 vs 다음 노드로 나갈 방향)"""
        if not (self.in_guide and step.guide and nxt and nxt.guide) or REVERSE in (step.drive, nxt.drive):
            return False
        if came is None or math.hypot(step.x - came[0], step.y - came[1]) < 0.05:
            return False          # 제자리 노드 (출발 노드 = 지금 선 자리): 들어온 방향이 없음
        if self.check_every:
            return True           # 노드마다 확인 (다음 노드가 있는 = 도착지가 아닌 노드)
        if self.check_xy and math.hypot(step.x - self.check_xy[0], step.y - self.check_xy[1]) < self.check_gap:
            return False          # 방금 인사 · 확인한 자리 근처
        h_in = math.atan2(step.y - came[1], step.x - came[0])
        h_out = math.atan2(nxt.y - step.y, nxt.x - step.x)
        return abs(wrap(h_out - h_in)) > self.check_turn

    def _guide_request(self, action, node, after):
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        self.io.direction('STOP')
        self.phase = 'guide_greet' if action == 'greet' else 'guide_watch'
        self.guide_state = 'running'
        self.guide_after = after
        self.io.guide({'action': action, 'node': node})

    def _late_check(self, pose):
        """다음 노드를 늦게 받음 (서버가 lock을 못 잡아 1칸씩 보냄): 도착 때는 꺾임을 몰랐으니 지금 판단.
        지금 선 노드가 꺾이는 곳이면 여기서 확인하고 이어 감 -> True"""
        step = self.cur
        if step is None or step.node_id != self.last_node or not len(self.queue):
            return False
        if not self._is_check(step, self.queue.steps[0], self.came_xy):
            return False
        self._begin_check(pose, self.came_xy)
        return True

    def _begin_check(self, pose, came=None):
        """손님 확인 노드: 멈춤 -> 왔던 쪽(손님)으로 돎 -> watch -> 다음 노드 쪽으로 돎 -> 이어 감
        (도착 yaw는 서버가 준 다음 노드 쪽이라 "+180도"면 꺾이는 노드에서 옆을 봄. 직전 노드 쪽을 본다)"""
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        self.io.direction('STOP')
        node = self.cur.node_id
        self.check_xy = (self.cur.x, self.cur.y)
        self.phase = 'guide_turn'
        if came is not None and math.hypot(came[0] - pose[0], came[1] - pose[1]) > 0.05:
            self.turn_target = math.atan2(came[1] - pose[1], came[0] - pose[0])
        else:
            self.turn_target = wrap(pose[2] + math.pi)
        self.io.log(f'{node}: 손님 확인 (뒤로 돎)')

        def after_back(p):
            self._guide_request('watch', node, after_watch)

        def after_watch(p):
            nxt = self.queue.steps[0] if len(self.queue) else None
            if nxt is None:  # 확인 노드가 줄 끝: 그대로 끝 처리
                self._after_node(p)
                return
            heading = math.atan2(nxt.y - p[1], nxt.x - p[0])
            self.phase = 'guide_turn'
            self.turn_target = wrap(heading + math.pi) if nxt.drive == REVERSE else heading
            self.guide_after = self._after_node

        self.guide_after = after_back

    def _guest_lost(self):
        node = self.cur.node_id if self.cur else self.last_node
        reason = f'guest_lost:{node}:{self.guide_detail or "not_seen"}'
        self.io.log(f'손님을 놓침 -> 멈춤 ({reason})')
        self.io.event(CMD_FAILED, self.queue.cmd_id, node, reason)
        self.queue.clear()
        self.cur = None
        self.in_guide = False
        self.guide_state = ''
        self.phase = 'idle'
        self.io.direction('STOP')

    def _stop_motion(self, keep=False, keep_dir=False):
        self.io.cmd_vel(0.0, 0.0)
        self.io.nav_cancel()
        if self.phase in ('align', 'precision', 'leave', 'tape_approach'):
            self.io.precision_cancel()
        if self.phase in ('guide_greet', 'guide_watch'):
            self.io.guide_cancel()
        if not keep:
            self.nav_state = ''
            self.prec_state = ''
            self.holding = False
            self.policy.update(0.0, '')
            self.io.lcd_override(None)
        if not keep_dir:
            self.io.direction('STOP')
