"""Docking 노드: fleet_agent가 맡긴 정밀 정차를 한다

fleet_agent <-> docking 인터페이스 (std_msgs/String, JSON)
  /docking/request  {"id", "node", "method": "dock" | "align" | "tape", ...precision_stops.yaml 값 (marker, markers, back)}
  /docking/result   {"id", "node", "ok": bool, "detail": str}
  /docking/cancel   {"id"}   -> 바로 멈추고 결과는 보내지 않는다

방법 -> 구현
  dock  : docking/marker_dock.py  Docker.dock(마커 marker의 목표 파일) -> back(m)만큼 후진   (D1~D4)
  align : 같은 Docker로 markers를 차례로 (안 보이면 돌며 찾기)                              (N14: 5 -> 6)
  tape  : docking/line_dock.py    LineDocker.run  E자 테이프로 바구니를 픽업대에              (P1)
  (leave = 노드를 떠날 때, precision_stops의 <노드>.leave)
  undock    : docking/undock.py   마커로 나올 자리까지 + 회전 + 방위각 정렬                  (D3/D4 -> N16/N15)
  tape_exit : docking/line_dock.py LineDocker.leave  앞으로 빼서 새 선에서 정지              (P1 -> 그다음 Nav2)
  마커 목표 파일은 혼자 실행으로 미리 저장: ros2 run fleet_agent marker_dock --record --id <마커>

- 정밀 정차는 시간이 걸리고 멈춰서 재는 동작이라 따로 스레드에서 돈다 (TF는 노드 executor가 받음)
- timeout_sec 안에 못 끝내면 멈추고 ok=True, detail="timeout"으로 끝낸다 (Nav2 위치 인정)
- 일하는 동안만 /cmd_vel을 쓴다 (fleet_agent는 그동안 멈춰 있음)
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16MultiArray
from tf2_ros import Buffer, TransformListener

from .docking import marker_dock as md
from .docking.line_dock import LineDocker
from .docking.undock import undock


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking')
        self.timeout = self.declare_parameter('timeout_sec', 40.0).value
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, '/docking/result', reliable)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/docking/request', self.on_request, reliable)
        self.create_subscription(String, '/docking/cancel', self.on_cancel, reliable)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.ir = {'v': None, 'n': 0}      # 바닥 센서 (tape · tape_exit)
        self.create_subscription(UInt16MultiArray, self.declare_parameter('ir_topic', '/ir_sensor/range').value,
                                 self.on_ir, 10)

        self.cam = None            # 처음 마커 도킹 때 구독 시작 (카메라는 camera_node가 열고 있음)
        self.job = None            # 지금 요청
        self.job_start = 0.0
        self.cancel_flag = False
        self.timed_out = False
        self.lock = threading.Lock()
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info('docking 노드 시작')

    # ------------------------------------------------------------ 요청
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            if self.job is not None:
                self.publish(req, False, f'다른 정밀 정차 중 ({self.job.get("node")})')
                return
            self.job = req
            self.job_start = time.monotonic()
            self.cancel_flag = False
            self.timed_out = False
        self.get_logger().info(f'{req.get("node")} {req.get("method")} 시작 {req}')
        threading.Thread(target=self.run, args=(req,), daemon=True).start()

    def on_ir(self, msg):
        self.ir['v'] = list(msg.data[:3])
        self.ir['n'] += 1

    def on_cancel(self, msg):
        with self.lock:
            if self.job is not None:
                self.cancel_flag = True
                self.get_logger().info(f'{self.job.get("node")} 취소')

    def watchdog(self):
        with self.lock:
            if self.job is not None and not self.cancel_flag and time.monotonic() - self.job_start > self.timeout:
                self.timed_out = True
                self.cancel_flag = True
                self.get_logger().warn(f'{self.job.get("node")} {self.timeout:.0f}초 넘김 -> 멈추고 Nav2 위치 인정')

    def cancelled(self):
        return self.cancel_flag

    # ------------------------------------------------------------ 실행 (스레드)
    def run(self, req):
        method = req.get('method')
        try:
            if method == 'dock':
                result = self.run_dock(req)
            elif method == 'align':
                result = self.run_align(req)
            elif method == 'tape':
                result = LineDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir).run(req)
            elif method == 'tape_exit':
                result = LineDocker(self, self.cmd_pub, self.cancelled, self.tf_buf, self.ir).leave(req)
            elif method == 'undock':
                result = self.run_undock(req)
            else:
                result = f'실패: 모르는 method {method}'
        except md.Cancelled:
            result = None
        except Exception as e:  # 카메라 · TF 문제 등
            result = f'실패: {e}'
        finally:
            self.cmd_pub.publish(Twist())
            md.close_log()

        with self.lock:
            timed_out = self.timed_out
            self.job = None
        if result is None and not timed_out:
            return  # 취소: 결과 안 보냄
        if timed_out:
            self.publish(req, True, 'timeout: Nav2 위치 인정')
        else:
            self.publish(req, result == '성공', result)

    def docker(self, marker_id, tag):
        md.load_cam_file()
        md.open_log(tag)
        if self.cam is None:
            self.cam = md.Camera(marker_id, self, spin=False)  # camera_node 영상 구독 (카메라를 쥐지 않음)
        self.cam.id = marker_id
        return md.Docker(self, self.cmd_pub, self.tf_buf, self.cam, moving=True, spin=False,
                         cancelled=self.cancelled)

    def run_dock(self, req):
        marker = int(req['marker'])
        dk = self.docker(marker, f'{req.get("node")}_dock')
        dk.wait_pose()
        result = dk.dock(md.target_file_of(marker))
        back = float(req.get('back', 0.0))
        if result == '성공' and back > 0.0:
            left = dk.back_off(back)
            md.log(f'[후진] {back * 100:.1f} cm (남은 {left * 1000:+.1f} mm)')
        return result

    def run_undock(self, req):
        dk = self.docker(int(req['marker']), f'{req.get("node")}_undock')
        return undock(self, self.cmd_pub, self.cancelled, self.tf_buf, dk.cam, req)

    def run_align(self, req):
        result = '실패: markers 없음'
        for marker in req.get('markers', []):
            dk = self.docker(int(marker), f'{req.get("node")}_align{marker}')
            dk.wait_pose()
            if not dk.find_marker():
                return f'실패: 마커 {marker}가 안 보임'
            result = dk.dock(md.target_file_of(int(marker)))
            if result != '성공':
                return f'마커 {marker}: {result}'
        return result

    # ------------------------------------------------------------
    def publish(self, req, ok, detail):
        out = {'id': req.get('id'), 'node': req.get('node'), 'ok': ok, 'detail': detail}
        self.result_pub.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f'{req.get("node")} 끝: {detail}')

    def close(self):
        self.cmd_pub.publish(Twist())
        if self.cam is not None:
            self.cam.close()


def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
