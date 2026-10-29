"""Fleet Agent: 메인 서비스 <-> 로봇 (fleet_msgs, README 2장 · 5장)

받음  /fleet/move · /fleet/control · /fleet/display      (서버, domain_bridge 거쳐서)
보냄  /fleet/state (주기) · /fleet/event (일이 생길 때)
로봇 안
  Nav2         navigate_to_pose 액션 (FORWARD)
  /cmd_vel     직선 후진 · 제자리 회전 (REVERSE, 방향 바꿀 때)
  Docking 노드 /docking/request -> /docking/result   (정밀 정차 dock / align / tape)
  Obstacle 노드 /obstacle/status <- (막힘 · 종류),  /obstacle/direction -> (지금 볼 방향)
  Guide 노드   /guide/request -> /guide/result      (안내: 손님 기다림 · 확인 · 효과). 없으면 바로 OK
  LCD          /lcd/text (std_msgs/String, LCD 드라이버가 받아 띄움)
위치는 TF map -> base_link
"""
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from fleet_msgs.msg import Control, Display, Move, RobotEvent, RobotState

from .core import BlockPolicy, Driver, ReverseControl, load_precision_stops


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FleetAgent(Node):
    def __init__(self):
        super().__init__('fleet_agent')
        p = self.declare_parameter
        self.robot_id = p('robot_id', 'pinky1').value
        self.start_node = p('start_node', '').value          # 켤 때 서 있는 도킹 (서버가 위치 확인에 씀)
        stops_file = p('precision_stops', '').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        nav_action = p('nav_action', 'navigate_to_pose').value
        state_hz = p('state_hz', 5.0).value
        self.battery_default = p('battery_default', 100.0).value  # 배터리를 못 읽으면 이 값 (%)
        battery_topic = p('battery_topic', '').value              # sensor_msgs/BatteryState, '' = 안 씀
        self.pass_text = p('pass_text', '지나갈게요').value

        stops = load_precision_stops(stops_file)
        if stops:
            self.get_logger().info(f'{self.robot_id}: 정밀 정차 노드 {sorted(stops)}')
        else:
            self.get_logger().info(f'{self.robot_id}: 정밀 정차 끔 (Nav2 · 후진 위치로만 정차)')

        rev = ReverseControl(speed=p('reverse_speed', 0.06).value, tol=p('reverse_tolerance', 0.03).value)
        policy = BlockPolicy(dynamic_wait=p('dynamic_wait_sec', 3.0).value, dynamic_ask=p('dynamic_ask_sec', 5.0).value,
                             static_wait=p('static_wait_sec', 2.0).value, unknown_wait=p('unknown_wait_sec', 8.0).value)
        self.driver = Driver(self, stops, pass_tol=p('pass_tolerance', 0.08).value,
                             yaw_tol=p('yaw_tolerance', 0.15).value, done_grace=p('done_grace_sec', 0.5).value,
                             reverse=rev, policy=policy, nav_retries=p('nav_retries', 1).value,
                             check_turn_deg=p('guide_check_turn_deg', 45.0).value,
                             check_gap=p('guide_check_gap_m', 0.8).value,
                             check_every=p('guide_check_every_node', False).value)
        self.driver.last_node = self.start_node
        self.driver.leave_from = self.start_node   # 켤 때 선 노드에 leave(언도킹 등)가 있으면 첫 출발 때 먼저

        # ---- fleet 토픽 (README 2.3 QoS)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(RobotState, '/fleet/state', best)
        self.event_pub = self.create_publisher(RobotEvent, '/fleet/event', reliable)
        self.create_subscription(Move, '/fleet/move', self.on_move, reliable)
        self.create_subscription(Control, '/fleet/control', self.on_control, reliable)
        self.create_subscription(Display, '/fleet/display', self.on_display, latched)

        # ---- 로봇 안
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lcd_pub = self.create_publisher(String, '/lcd/text', latched)
        self.dock_req_pub = self.create_publisher(String, '/docking/request', reliable)
        self.dock_cancel_pub = self.create_publisher(String, '/docking/cancel', reliable)
        self.dir_pub = self.create_publisher(String, '/obstacle/direction', latched)
        self.create_subscription(String, '/docking/result', self.on_dock_result, reliable)
        self.create_subscription(String, '/obstacle/status', self.on_obstacle, 10)
        self.guide_req_pub = self.create_publisher(String, '/guide/request', reliable)
        self.guide_cancel_pub = self.create_publisher(String, '/guide/cancel', reliable)
        self.create_subscription(String, '/guide/result', self.on_guide_result, reliable)
        if battery_topic:
            self.create_subscription(BatteryState, battery_topic, self.on_battery, 10)

        self.nav = ActionClient(self, NavigateToPose, nav_action)
        self.nav_goal = None
        self.nav_seq = 0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.battery = float(self.battery_default)
        self.dock_req_id = 0
        self.guide_req_id = 0
        self.lcd_text = ''
        self.lcd_count = 0
        self.lcd_override_text = None
        self.last_dir = ''
        self.pose_warned = False

        self.create_timer(0.05, self.tick)                 # 주행 20 Hz
        self.create_timer(1.0 / state_hz, self.publish_state)
        self.create_timer(1.0, self.lcd_tick)               # 카운트다운
        self.get_logger().info(f'{self.robot_id} fleet_agent 시작 (시작 노드 {self.start_node or "없음"})')

    # ================================================================ 서버 -> 로봇
    def on_move(self, msg):
        wps = [(w.node_id, w.x, w.y, w.yaw) for w in msg.waypoints]
        guide = bool(getattr(msg, 'guide', False))
        goal = getattr(msg, 'goal_node', '')
        append = msg.mode == Move.APPEND
        self.get_logger().info(f'Move cmd {msg.cmd_id} {"APPEND" if append else "REPLACE"} '
                               f'{"REVERSE" if msg.drive == Move.REVERSE else "FORWARD"}{" 안내" if guide else ""} '
                               f'{[w[0] for w in wps]} -> 도착지 {goal or "?"}')
        self.driver.on_move(msg.cmd_id, append, msg.drive, wps, guide, goal)

    def on_control(self, msg):
        self.get_logger().info(f'Control {["PAUSE", "RESUME", "CANCEL"][msg.command] if msg.command < 3 else msg.command}')
        self.driver.on_control(msg.command)

    def on_display(self, msg):
        self.lcd_text = msg.text
        self.lcd_count = int(msg.countdown_sec)
        self.show_lcd()

    # ================================================================ 로봇 -> 서버
    def publish_state(self):
        m = RobotState()
        m.robot_id = self.robot_id
        m.stamp = self.get_clock().now().to_msg()
        pose = self.pose()
        if pose:
            m.x, m.y, m.yaw = pose
        m.state = self.driver.run_state()
        m.cmd_id = self.driver.queue.cmd_id
        m.last_node = self.driver.last_node
        m.battery = float(self.battery)
        self.state_pub.publish(m)

    def event(self, etype, cmd_id, node, reason):  # Driver io
        m = RobotEvent()
        m.robot_id = self.robot_id
        m.stamp = self.get_clock().now().to_msg()
        m.type = etype
        m.cmd_id = int(cmd_id)
        m.node_id = node
        m.reason = reason
        self.event_pub.publish(m)
        name = {0: 'NODE_REACHED', 1: 'CMD_DONE', 2: 'CMD_FAILED', 4: 'BLOCKED'}.get(etype, str(etype))
        self.get_logger().info(f'event {name} {node} {reason}')

    # ================================================================ 주기
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.driver.tick(now, self.pose())

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.0))
        except Exception as e:  # TF가 아직 없음 (amcl 초기 위치 전)
            if not self.pose_warned:
                self.get_logger().warn(f'위치(TF {self.map_frame} -> {self.base_frame})를 아직 못 받음: {e}')
                self.pose_warned = True
            return None
        tr = t.transform.translation
        return tr.x, tr.y, quat_to_yaw(t.transform.rotation)

    def odom_yaw(self):
        """odom 기준 방향 (축 고정 후진의 방향 유지용. AMCL 보정이 안 섞임). 없으면 None"""
        try:
            t = self.tf.lookup_transform('odom', self.base_frame, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        return quat_to_yaw(t.transform.rotation)

    # ================================================================ Driver io
    def nav_go(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('Nav2 navigate_to_pose 서버가 없음')
            self.driver.on_nav_result(False)
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = yaw_to_quat(float(yaw))
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        goal.pose = ps
        self.nav_seq += 1
        seq = self.nav_seq
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._nav_accepted(f, seq))

    def _nav_accepted(self, fut, seq):
        handle = fut.result()
        if seq != self.nav_seq:  # 그사이 새 목표가 나감 -> 옛 목표는 버림
            if handle and handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle or not handle.accepted:
            self.get_logger().warn('Nav2가 목표를 받지 않음')
            self.driver.on_nav_result(False)
            return
        self.nav_goal = handle
        handle.get_result_async().add_done_callback(lambda f: self._nav_done(f, seq))

    def _nav_done(self, fut, seq):
        if seq != self.nav_seq:
            return
        status = fut.result().status
        self.nav_goal = None
        if status == GoalStatus.STATUS_CANCELED:
            return  # 우리가 멈춘 것 (정지 · 장애물 · 새 목표)
        self.driver.on_nav_result(status == GoalStatus.STATUS_SUCCEEDED)

    def nav_cancel(self):
        self.nav_seq += 1  # 오는 결과는 무시
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None

    def cmd_vel(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def precision(self, req):
        self.dock_req_id += 1
        req = dict(req)
        req['id'] = self.dock_req_id
        self.dock_req_pub.publish(String(data=json.dumps(req)))
        self.get_logger().info(f'정밀 정차 요청 {req}')

    def precision_cancel(self):
        self.dock_cancel_pub.publish(String(data=json.dumps({'id': self.dock_req_id})))

    def on_dock_result(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        if r.get('id') != self.dock_req_id:
            return
        self.get_logger().info(f'정밀 정차 결과 {r}')
        self.driver.on_precision_result(bool(r.get('ok')), r.get('detail', ''))

    def guide(self, req):
        self.guide_req_id += 1
        req = dict(req)
        req['id'] = self.guide_req_id
        if self.guide_req_pub.get_subscription_count() == 0:  # Guide 노드를 안 띄움 -> 손님 확인 없이 진행
            self.get_logger().warn(f'Guide 노드 없음 -> {req["action"]} 건너뜀')
            if req['action'] in ('greet', 'watch'):
                self.driver.on_guide_result(True, 'no_guide_node')
            return
        self.guide_req_pub.publish(String(data=json.dumps(req)))
        self.get_logger().info(f'안내 요청 {req}')

    def guide_cancel(self):
        self.guide_cancel_pub.publish(String(data=json.dumps({'id': self.guide_req_id})))

    def on_guide_result(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        if r.get('id') != self.guide_req_id:
            return
        self.get_logger().info(f'안내 결과 {r}')
        self.driver.on_guide_result(bool(r.get('ok')), r.get('detail', ''))

    def direction(self, text):
        if text != self.last_dir:
            self.last_dir = text
            self.dir_pub.publish(String(data=text))

    def on_obstacle(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.driver.set_obstacle(s.get('kind', '') if s.get('blocked') else '')

    def lcd_override(self, text):
        if text != self.lcd_override_text:
            self.lcd_override_text = text
            self.show_lcd()

    def log(self, text):
        self.get_logger().info(text)

    # ================================================================ LCD · 배터리
    def lcd_tick(self):
        if self.lcd_count > 0:
            self.lcd_count -= 1
            self.show_lcd()

    def show_lcd(self):
        if self.lcd_override_text:
            text = self.lcd_override_text
        else:
            text = self.lcd_text + (f' ({self.lcd_count})' if self.lcd_count > 0 else '')
        self.lcd_pub.publish(String(data=text))

    def on_battery(self, msg):
        pct = msg.percentage  # BatteryState 규격: 0.0 ~ 1.0
        if math.isnan(pct):
            return
        self.battery = max(0.0, min(100.0, pct * 100.0))


def main():
    rclpy.init()
    node = FleetAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
