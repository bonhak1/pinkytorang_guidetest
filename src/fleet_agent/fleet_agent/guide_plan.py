"""안내 시험용 경로 계획 (ROS를 모른다): 서버 없이 서버와 같은 안내 Move를 만든다

서버(main_service)가 하는 것 중 안내에 필요한 만큼만 똑같이 따라 한다
  - 길 찾기   : MapManager::shortestPathAvoid (다익스트라, 거리 = 노드 사이 직선거리)
  - 안내 자리 : guide_spots {T3: N6 ...} -> 테이블 칸에 안 들어가고 그 노드에서 테이블 쪽(face)을 보고 섬
  - yaw      : TrafficManager::publish yawAt (지나가는 노드 = 다음 노드 쪽, 후진이면 +180도,
               도착지 = face 노드 쪽 > 노드 yaw > 들어온 방향)
  - 후진     : node_graph reverse 구간 + entry REVERSE 포켓(도킹)으로 들어가는 길
  - 끊어 보내기 : lookahead 노드만큼 먼저 보내고, 로봇이 노드에 닿을 때마다 APPEND (서버 lock 흉내)
               방향이 바뀌는 곳에서 Move를 나눔 (같은 cmd_id, APPEND)

  guide_sim (PC 시뮬) · guide_run (실제 로봇) · test_guide 가 같이 쓴다
"""
import math
import os
from dataclasses import dataclass, field

import yaml

FORWARD = 0
REVERSE = 1

# main_service.yaml guide_spots (서버 설정을 못 읽을 때 기본값)
DEFAULT_GUIDE_SPOTS = {'T1': 'N3', 'T2': 'N3', 'T3': 'N6', 'T4': 'N6', 'T5': 'N9', 'T6': 'N9'}

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_SRC = os.path.normpath(os.path.join(_HERE, '..', '..'))     # src/fleet_agent/fleet_agent -> src


def _share(pkg, *rel):
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory(pkg), *rel)
        return p if os.path.exists(p) else ''
    except Exception:  # noqa: BLE001  (ROS 없는 PC · 빌드 전)
        return ''


def default_graph_path():
    """node_graph.yaml: 설치된 fleet_msgs share -> 없으면 레포 안 (src/fleet_msgs/docs)"""
    p = _share('fleet_msgs', 'docs', 'node_graph.yaml')
    if p:
        return p
    p = os.path.join(_REPO_SRC, 'fleet_msgs', 'docs', 'node_graph.yaml')
    return p if os.path.exists(p) else ''


def load_guide_spots(path=''):
    """main_service.yaml의 guide_spots. 못 읽으면 기본값"""
    cands = [path] if path else [os.path.join(_REPO_SRC, 'main_service', 'config', 'main_service.yaml'),
                                 _share('main_service', 'config', 'main_service.yaml')]
    for c in cands:
        if c and os.path.exists(c):
            try:
                with open(c, encoding='utf-8') as f:
                    spots = (yaml.safe_load(f) or {}).get('guide_spots')
                if isinstance(spots, dict) and spots:
                    return {str(k): str(v) for k, v in spots.items()}
            except Exception:  # noqa: BLE001
                pass
    return dict(DEFAULT_GUIDE_SPOTS)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GuideMap:
    def __init__(self, path=''):
        path = path or default_graph_path()
        if not path:
            raise FileNotFoundError('node_graph.yaml을 못 찾음 (--graph로 지정)')
        self.file = path
        with open(path, encoding='utf-8') as f:
            g = yaml.safe_load(f)
        self.nodes = {n['id']: n for n in g['nodes']}
        self.rev_edges = set()
        for chain in g.get('reverse') or []:
            for a, b in zip(chain, chain[1:]):
                self.rev_edges.add((a, b))

    def xy(self, n):
        return self.nodes[n]['x'], self.nodes[n]['y']

    def dist(self, a, b):
        (ax, ay), (bx, by) = self.xy(a), self.xy(b)
        return math.hypot(bx - ax, by - ay)

    def heading(self, a, b):
        (ax, ay), (bx, by) = self.xy(a), self.xy(b)
        return math.atan2(by - ay, bx - ax)

    def path(self, a, b, ban=()):
        """서버 shortestPathAvoid와 같은 다익스트라 (ban = 지나가지 않을 노드, 도착지는 예외)"""
        if a not in self.nodes or b not in self.nodes:
            return []
        if a == b:
            return [a]
        dist = {n: 1e9 for n in self.nodes}
        prev = {}
        done = set()
        dist[a] = 0.0
        while True:
            u, best = '', 1e9
            for n in self.nodes:                      # 서버와 같은 순서 (파일 순서) -> 같은 길
                if n not in done and dist[n] < best:
                    u, best = n, dist[n]
            if not u or u == b:
                break
            done.add(u)
            for v in self.nodes[u].get('next', []):
                if v != b and v in ban:
                    continue
                d = dist[u] + self.dist(u, v)
                if d < dist[v]:
                    dist[v] = d
                    prev[v] = u
        if b not in prev:
            return []
        out = [b]
        while out[-1] != a:
            out.append(prev[out[-1]])
        return out[::-1]

    def edge_reverse(self, a, b):
        """a -> b를 후진으로 가는가 (reverse 구간, 또는 후진 진입 포켓(도킹)으로 들어감)"""
        if a == b:
            return False
        if (a, b) in self.rev_edges:
            return True
        nb = self.nodes[b]
        return bool(nb.get('pocket')) and str(nb.get('entry', 'FORWARD')).upper() == 'REVERSE'

    def length(self, nodes):
        return sum(self.dist(a, b) for a, b in zip(nodes, nodes[1:]))


@dataclass
class Wp:
    node: str
    x: float
    y: float
    yaw: float
    drive: int


@dataclass
class Plan:
    """한 번의 이동 (안내 또는 복귀)"""
    nodes: list            # 출발 노드부터 도착지까지
    goal: str
    guide: bool
    face: str = ''         # 도착지에서 이 노드 쪽을 봄 (안내 = 테이블)
    table: str = ''
    wps: list = field(default_factory=list)   # [Wp] (nodes와 같은 길이)

    def text(self):
        out = []
        for w in self.wps:
            out.append(w.node + ('(R)' if w.drive == REVERSE else ''))
        return ' -> '.join(out)


def build_wps(g, nodes, goal, face=''):
    """서버 TrafficManager::publish의 yawAt · 후진 판단을 따라 한다. nodes[0] = 지금 선 노드"""
    rev = [False] + [g.edge_reverse(a, b) for a, b in zip(nodes, nodes[1:])]
    wps = []
    for i, nid in enumerate(nodes):
        n = g.nodes[nid]
        last = i + 1 >= len(nodes)
        if last and nid == goal and face and face in g.nodes:
            yaw = g.heading(nid, face)
        elif last and nid == goal and 'yaw' in n:
            yaw = float(n['yaw'])
        else:
            back = False
            if not last:
                yaw = g.heading(nid, nodes[i + 1])
                back = rev[i + 1]
            elif i > 0:
                yaw = g.heading(nodes[i - 1], nid)
                back = rev[i]
            else:
                yaw = float(n.get('yaw', 0.0))
            if back:
                yaw = wrap(yaw + math.pi)
        wps.append(Wp(nid, float(n['x']), float(n['y']), yaw, REVERSE if rev[i] else FORWARD))
    # 첫 노드(지금 자리)의 drive는 다음 길과 같게 (그 자리에서 출발할 방향)
    if len(wps) >= 2:
        wps[0].drive = wps[1].drive
    return wps


def guide_plan(g, start, table, spots=None):
    """입구(start)에서 table 안내: guide_spots 노드까지, 마지막엔 테이블 쪽을 봄"""
    spots = DEFAULT_GUIDE_SPOTS if spots is None else spots
    if table not in g.nodes:
        raise ValueError(f'node_graph에 없는 테이블: {table}')
    goal, face = (spots[table], table) if spots.get(table) in g.nodes else (table, '')
    nodes = g.path(start, goal)
    if not nodes:
        raise ValueError(f'{start} -> {goal} 길이 없음')
    p = Plan(nodes, goal, True, face, table)
    p.wps = build_wps(g, nodes, goal, face)
    return p


def return_plan(g, start, dock):
    """안내 끝 -> 도킹(dock)으로 복귀 (guide=false). 도킹은 후진 진입"""
    nodes = g.path(start, dock)
    if not nodes:
        raise ValueError(f'{start} -> {dock} 길이 없음')
    p = Plan(nodes, dock, False)
    p.wps = build_wps(g, nodes, dock)
    return p


def plan_to(g, start, goal, guide, face=''):
    """아무 노드로 (guide_run --to)"""
    nodes = g.path(start, goal)
    if not nodes:
        raise ValueError(f'{start} -> {goal} 길이 없음')
    p = Plan(nodes, goal, guide, face)
    p.wps = build_wps(g, nodes, goal, face)
    return p


@dataclass
class MoveSpec:
    """fleet_msgs/Move 한 개 (ROS 없이)"""
    cmd_id: int
    append: bool
    drive: int
    goal: str
    guide: bool
    waypoints: list        # [(node, x, y, yaw)]

    def nodes(self):
        return [w[0] for w in self.waypoints]

    def text(self):
        return (f'cmd {self.cmd_id} {"APPEND " if self.append else "REPLACE"} '
                f'{"REVERSE" if self.drive == REVERSE else "FORWARD"}{" 안내" if self.guide else ""} '
                f'{self.nodes()} -> 도착지 {self.goal}')


class MoveFeeder:
    """서버처럼 끊어 보낸다: 처음엔 지금 노드 + lookahead 노드, 로봇이 노드에 닿을 때마다 그만큼 앞까지 APPEND
    lookahead <= 0 = 전부 한 번에. 방향(drive)이 바뀌는 곳에서 Move를 나눈다 (같은 cmd_id, APPEND)"""

    def __init__(self, plan, cmd_id, lookahead=2):
        self.plan = plan
        self.cmd_id = cmd_id
        self.lookahead = lookahead if lookahead and lookahead > 0 else len(plan.wps)
        self.sent = 0            # 보낸 waypoint 수
        self.reached = 0         # 닿은 waypoint 수 (지금 선 노드 = 0번은 닿은 것으로 본다)

    def _upto(self):
        return min(len(self.plan.wps), self.reached + 1 + self.lookahead)

    def _emit(self, upto, first):
        out = []
        wps = self.plan.wps[self.sent:upto]
        i = 0
        while i < len(wps):
            j = i
            while j + 1 < len(wps) and wps[j + 1].drive == wps[i].drive:
                j += 1
            seg = wps[i:j + 1]
            out.append(MoveSpec(self.cmd_id, not (first and i == 0), seg[0].drive, self.plan.goal,
                                self.plan.guide, [(w.node, w.x, w.y, w.yaw) for w in seg]))
            i = j + 1
        self.sent = upto
        return out

    def first(self):
        return self._emit(self._upto(), True)

    def on_reached(self, node):
        """NODE_REACHED(node) -> 더 보낼 APPEND (없으면 [])"""
        for k in range(self.reached, self.sent):
            if self.plan.wps[k].node == node:
                self.reached = k
                break
        else:
            return []
        upto = self._upto()
        return self._emit(upto, False) if upto > self.sent else []

    def done_sending(self):
        return self.sent >= len(self.plan.wps)
