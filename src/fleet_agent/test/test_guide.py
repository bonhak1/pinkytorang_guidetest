"""안내 로봇 시험 (PC, ROS 없이):  python3 -m pytest test/test_guide.py -v

진짜 node_graph · guide_spots로 서버와 같은 안내 Move를 만들어(guide_plan) 진짜 core.Driver를 돌린다(guide_sim).
  1. 경로 계획       : 안내 자리 · 마지막 yaw(테이블 쪽) · 복귀 후진 진입 · 끊어 보내기
  2. 안내 흐름       : 인사 -> 꺾이는 노드 확인 -> 도착 효과, 테이블 6개 전부
  3. 실패 · 끼어들기 : 손님 놓침(확인 · 인사), PAUSE/RESUME, 사람 막힘
  4. 시험 도구       : fake_guide 판정 규칙, guide_run Runner를 시뮬 로봇에 붙여서
  5. 알려진 문제     : xfail(strict) — 고치면 XPASS로 실패하니 그때 표시를 지운다
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet_agent.core import CMD_DONE, CMD_FAILED, NODE_REACHED, BlockPolicy, Driver, ReverseControl  # noqa: E402
from fleet_agent.fake_guide import FakeRules  # noqa: E402
from fleet_agent.guide_plan import (FORWARD, REVERSE, GuideMap, MoveFeeder, guide_plan, load_guide_spots,  # noqa: E402
                                    return_plan, wrap)
from fleet_agent.guide_run import Runner  # noqa: E402
from fleet_agent.guide_sim import Guest, SimRobot, run_scenario  # noqa: E402

G = GuideMap()
SPOTS = load_guide_spots()
TABLES = sorted(SPOTS)


# ================================================================ 1. 경로 계획
@pytest.mark.parametrize('table', TABLES)
def test_plan_goes_to_guide_spot_and_faces_table(table):
    p = guide_plan(G, 'D1', table, SPOTS)
    assert p.nodes[0] == 'D1' and p.goal == SPOTS[table] and p.nodes[-1] == p.goal and p.guide
    for a, b in zip(p.nodes, p.nodes[1:]):
        assert b in G.nodes[a]['next'], f'{a}->{b} 엣지 없음'
    assert all(w.drive == FORWARD for w in p.wps), '안내는 전진만'
    assert abs(wrap(p.wps[-1].yaw - G.heading(p.goal, table))) < 1e-6, '도착지에서 테이블 쪽을 봐야 함'
    assert p.goal != table, '안내는 테이블 칸에 서지 않음 (지나갈 수는 있음: 왼쪽 줄 -> 가운데 줄은 T1/T3/T5를 지남)'


def test_plan_return_backs_into_dock():
    b = return_plan(G, 'N6', 'D1')
    assert b.nodes[-1] == 'D1' and not b.guide
    assert b.wps[-1].drive == REVERSE and all(w.drive == FORWARD for w in b.wps[:-1])
    assert abs(wrap(b.wps[-1].yaw - G.nodes['D1']['yaw'])) < 1e-6


@pytest.mark.parametrize('look', [0, 1, 2, 3])
def test_feeder_sends_whole_route_in_chunks(look):
    p = return_plan(G, 'N9', 'D1')            # 전진 + 후진 섞인 길
    f = MoveFeeder(p, 7, look)
    moves = f.first()
    assert not moves[0].append and all(m.append for m in moves[1:])
    for w in p.wps:
        moves += f.on_reached(w.node)
    assert all(m.cmd_id == 7 for m in moves)
    assert [n for m in moves for n in m.nodes()] == p.nodes
    assert all(len({p.wps[p.nodes.index(n)].drive for n in m.nodes()}) == 1 for m in moves), 'Move 하나에 방향 하나'
    if look:
        assert len(moves[0].nodes()) <= look + 1
    assert f.done_sending()


# ================================================================ 2. 안내 흐름
@pytest.mark.parametrize('table', TABLES)
def test_guide_each_table_ok(table):
    r = run_scenario(G, table, spots=SPOTS)
    assert r.ok and r.back_ok, r.failed
    assert r.greet == ['D1'], '출발 전 입구에서 한 번만 인사'
    assert r.arrive == [r.plan.goal], '도착 효과는 도착지에서 한 번'
    assert r.yaw_err_deg < 10
    assert r.checks, '입구 -> 홀은 꺾이는 곳이 있어 확인이 최소 한 번'
    assert set(r.checks) <= set(r.plan.nodes[1:-1]), '확인은 지나가는 노드에서만'
    gl = [a for _, a, *_ in r.robot.guides]
    assert gl[0] == 'greet' and gl[-1] == 'arrive' and gl.count('arrive') == 1
    dones = [n for _, et, n, _ in r.robot.events if et == CMD_DONE]
    assert dones[0] == r.plan.goal, '안내 중간(끊어 보낸 끝)에서는 CMD_DONE 안 함'


def test_return_never_greets_or_checks():
    r = run_scenario(G, 'T5', spots=SPOTS)
    t_done = [t for t, et, n, _ in r.robot.events if et == CMD_DONE][0]
    assert [a for t, a, *_ in r.robot.guides if t > t_done + 0.01] == []


def test_check_gap_skips_check_near_greet():
    # check_gap을 크게 하면 입구(인사 자리) 근처 N1 확인은 건너뜀
    r = run_scenario(G, 'T3', spots=SPOTS, check_gap=1.2)
    assert 'N1' not in r.checks and r.ok


def test_check_turn_threshold():
    r = run_scenario(G, 'T5', spots=SPOTS, check_turn=100.0)    # 100도 넘게 꺾이는 곳 없음
    assert r.checks == [] and r.ok


# ================================================================ 3. 실패 · 끼어들기
def test_guest_lost_at_check_stops_there_then_returns():
    r = run_scenario(G, 'T5', spots=SPOTS, guest=Guest({'N3'}, wait_sec=3.0))
    assert not r.ok and r.failed == 'guest_lost:N3:not_seen'
    assert r.end_node == 'N3' and r.arrive == []
    fails = [(n, why) for _, et, n, why in r.robot.events if et == CMD_FAILED]
    assert fails == [('N3', 'guest_lost:N3:not_seen')]
    assert r.back_ok, '놓친 자리에서 도킹으로 복귀'


def test_greet_fail_does_not_move():
    r = run_scenario(G, 'T3', spots=SPOTS, guest=Guest(greet_fail=True, wait_sec=2.0), back=False)
    assert r.failed == 'guest_lost:D1:not_seen'
    assert [n for _, et, n, _ in r.robot.events if et == NODE_REACHED] == []
    assert r.robot.dist < 1e-6


def test_pause_resume_mid_guide():
    r = run_scenario(G, 'T3', spots=SPOTS, pause_at=('N2', 3.0))
    assert r.ok and r.arrive == ['N6']
    base = run_scenario(G, 'T3', spots=SPOTS)
    assert r.sec >= base.sec + 2.5


def test_pause_during_watch_asks_again():
    # 확인(watch) 중 PAUSE -> RESUME 하면 watch를 다시 요청
    g = Guest(react_sec=3.0)
    r = run_scenario(G, 'T3', spots=SPOTS, guest=g, pause_at=('N1', 1.0))
    assert r.ok
    assert [a for _, a, n, *_ in r.robot.guides if n == 'N1'].count('watch') >= 1


def test_person_blocks_briefly_no_blocked_report():
    # 사람이 4초 막음: 3초 뒤 LCD '지나갈게요', 8초 전에 비키면 BLOCKED 안 보냄
    r = run_scenario(G, 'T3', spots=SPOTS, obstacle=('N2', 4.0, 'DYNAMIC'))
    assert r.ok and r.blocked == []
    assert any('지나갈게요' in text for _, kind, text in r.robot.lines if kind == 'lcd')


def test_object_blocks_reports_blocked():
    r = run_scenario(G, 'T3', spots=SPOTS, obstacle=('N2', 4.0, 'STATIC'))
    assert r.ok and r.blocked and r.blocked[0].startswith('STATIC')


# ================================================================ 4. 시험 도구
def test_fake_guide_rules():
    f = FakeRules(['N3'], greet_fail=True, delay=0.5, fail_delay=2.0)
    assert f.decide('greet', 'D1') == (False, 2.0, 'not_seen')
    assert f.decide('greet', 'D1')[0] is True            # 한 번만 실패 (다시 안내하면 통과)
    assert f.decide('watch', 'N1') == (True, 0.5, 'guest_seen')
    assert f.decide('watch', 'N3')[0] is False
    assert f.decide('watch', 'N3')[0] is True
    always = FakeRules(['N3'], lost_once=False)
    assert not always.decide('watch', 'N3')[0] and not always.decide('watch', 'N3')[0]


class RunnerRig:
    """guide_run.Runner를 시뮬 로봇에 붙임 (ROS 대신)"""

    def __init__(self, targets, guest=None, **kw):
        sx, sy = G.xy('D1')
        self.robot = SimRobot(sx, sy, G.nodes['D1']['yaw'], guest or Guest())
        self.drv = Driver(self.robot, {}, reverse=ReverseControl(speed=0.08), policy=BlockPolicy())
        self.drv.last_node = 'D1'
        self.robot.driver = self.drv
        self.log = []
        self.ctl = []
        rig = self

        class Io:
            def send_move(self, m):
                rig.drv.on_move(m.cmd_id, m.append, m.drive, m.waypoints, m.guide, m.goal)

            def send_control(self, c):
                rig.ctl.append(c)        # ROS처럼 다음 주기에 (콜백 안에서 바로 넣지 않음)

            def log(self, text):
                rig.log.append(text)

            def pose(self):
                return rig.robot.x, rig.robot.y, rig.robot.yaw

        self.runner = Runner(Io(), G, targets, spots=SPOTS, seat_sec=1.0, gap_sec=0.5, cmd_base=1000, **kw)
        self.robot.on_event = self.runner.on_event
        orig_guide, orig_step = self.robot.guide, self.robot.step

        def guide(req):
            self.runner.on_guide_request(req['action'], req['node'])
            orig_guide(req)

        def step(dt):
            pend = self.robot.guide_pending
            orig_step(dt)
            if pend and self.robot.guide_pending is None and self.robot.t >= pend[0]:
                self.runner.on_guide_result(pend[2], pend[3])

        self.robot.guide = guide
        self.robot.step = step

    def run(self, limit=600.0, dt=0.05):
        self.runner.begin(0.0)
        while not self.runner.done() and self.robot.t < limit:
            self.runner.tick(self.robot.t)
            for c in self.ctl:
                self.drv.on_control(c)
            self.ctl = []
            self.drv.tick(self.robot.t, (self.robot.x, self.robot.y, self.robot.yaw))
            self.robot.step(dt)
        return self.runner


def test_guide_run_runner_sequence_with_lost_guest():
    rn = RunnerRig([('table', 'T3'), ('table', 'T5')], Guest({'N3'}, wait_sec=2.0, once=True)).run()
    assert [r.label for r in rn.runs] == ['T3', 'T5']
    t3, t5 = rn.runs
    assert t3.result == 'guest_lost:N3:not_seen' and t3.back == 'OK'   # N3에서 놓침 -> 그 자리에서 복귀
    assert t5.result == 'OK' and t5.arrive == 'N9' and t5.back == 'OK'  # 두 번째는 N3에서도 찾음
    assert t5.greet_sec > 0.5 and [c[0] for c in t5.checks] == ['N1', 'N3']
    rep = rn.report()
    assert 'guest_lost:N3' in rep and 'T5' in rep


def test_guide_run_runner_hold_and_cancel():
    rig = RunnerRig([('table', 'T3')], hold=('N1', 3.0))
    rn = rig.run()
    assert rn.runs[0].result == 'OK' and rn.runs[0].arrive == 'N6'
    assert any('늦게 줌' in line for line in rig.log)
    rn2 = RunnerRig([('table', 'T5')], cancel_at='N2').run()
    assert rn2.runs[0].result == 'CANCEL' and rn2.runs[0].back == 'OK'
    assert rn2.runs[0].arrive == ''


# ================================================================ 5. 알려진 문제 (고치면 xfail 표시를 지운다)
@pytest.mark.xfail(strict=True, reason='core._begin_check가 "지금 yaw + 180도"로 돎. 꺾이는 노드에서는 yaw가 '
                                       '다음 노드 쪽이라 손님(왔던 쪽) 대신 옆(약 90도)을 봄 -> guide-fixes.patch')
@pytest.mark.parametrize('table', ['T1', 'T3', 'T5'])
def test_known_watch_faces_the_guest(table):
    r = run_scenario(G, table, spots=SPOTS)
    assert r.look_err_deg and max(r.look_err_deg) < 30, f'손님 쪽과 {r.look_err_deg}도 차이'


@pytest.mark.xfail(strict=True, reason='core._reached가 다음 노드를 아직 모르면(서버가 1칸씩 보냄 = lock 경쟁) '
                                       '꺾임 판단을 못 해 확인을 통째로 건너뜀 -> guide-fixes.patch')
def test_known_check_even_when_server_sends_one_node_at_a_time():
    two = run_scenario(G, 'T5', spots=SPOTS, lookahead=2)
    one = run_scenario(G, 'T5', spots=SPOTS, lookahead=1)
    assert one.ok and one.checks == two.checks
