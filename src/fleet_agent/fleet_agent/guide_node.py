"""Guide 노드: 안내 로봇의 손님 확인 (카메라 + YOLO) · 효과. fleet_agent가 요청하고 결과를 받는다.

받음  /guide/request  std_msgs/String JSON {id, action, node}
        action = greet : 앞(카메라)으로 손님을 기다림 -> 찾으면 환영 효과 + ok
                 watch : (fleet_agent가 뒤로 돌려 놓은 상태) 손님이 보이는지 -> ok / 못 찾음
                 arrive: 도착 효과만 (결과 안 보냄)
      /guide/cancel   {id}  기다리던 것 그만
      camera_topic    sensor_msgs/Image (bgr8 / rgb8) 또는 sensor_msgs/CompressedImage
보냄  /guide/result   {id, ok, detail}

- YOLO는 요청을 처리하는 동안에만, infer_hz로 가장 최근 프레임 하나씩 돌린다 (CPU 아끼기, 주행 노드와 다른 프로세스)
- confirm 번 연속으로 보이면 "찾음". wait_sec 안에 못 찾으면 ok=false (fleet_agent가 서버에 guest_lost)
- 모델 · ultralytics가 없으면 require_detector=false일 때 바로 ok (detail no_detector) -> 카메라 없이도 안내 흐름 시험 가능
"""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from .guide_effects import play


class GuideNode(Node):
    def __init__(self):
        super().__init__('guide')
        p = self.declare_parameter
        self.camera_topic = p('camera_topic', '/camera/image_raw').value
        self.compressed = p('camera_compressed', False).value
        self.model_path = p('model_path', '').value
        self.class_name = p('class_name', 'PerfectFigure').value
        self.conf = float(p('conf', 0.5).value)
        self.infer_hz = float(p('infer_hz', 4.0).value)
        self.wait_sec = float(p('wait_sec', 30.0).value)
        self.confirm = int(p('confirm', 2).value)
        self.effects = p('effects', True).value
        self.require_detector = p('require_detector', False).value
        self.welcome_first = p('welcome_on_request', True).value   # greet가 오면(테이블 선택) 바로 환영

        self.frame = None
        self.frame_t = 0.0
        self.frame_lock = threading.Lock()
        self.job = None              # (id, action, node, cancel_event)
        self.model, self.class_id = self.load_model()

        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, '/guide/result', rel)
        self.create_subscription(String, '/guide/request', self.on_request, rel)
        self.create_subscription(String, '/guide/cancel', self.on_cancel, rel)
        img_type = CompressedImage if self.compressed else Image
        self.create_subscription(img_type, self.camera_topic, self.on_image, qos_profile_sensor_data)   # camera_node = best effort
        self.get_logger().info(f'guide 노드 시작 (카메라 {self.camera_topic}, '
                               f'{"YOLO " + self.class_name if self.model else "감지기 없음"})')

    # ---------------------------------------------------------------- 준비
    def load_model(self):
        if not self.model_path:
            self.get_logger().warn('model_path 없음 -> 손님 확인 없이 바로 OK')
            return None, None
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
            names = model.names
            items = names.items() if isinstance(names, dict) else enumerate(names)
            cid = next(k for k, v in items if v == self.class_name)
            return model, cid
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'YOLO 모델 못 읽음 ({e}) -> 손님 확인 없이 바로 OK')
            return None, None

    def on_image(self, msg):
        with self.frame_lock:
            self.frame = msg
            self.frame_t = time.monotonic()

    def to_bgr(self, msg):
        if self.compressed:
            import cv2
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img

    # ---------------------------------------------------------------- 요청
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        rid, action, node = req.get('id'), req.get('action'), req.get('node', '')
        if self.job:
            self.job[3].set()            # 새 요청이 오면 이전 것은 그만
        if action == 'arrive':
            self.effect('arrived')
            return
        if action not in ('greet', 'watch'):
            return
        if action == 'greet' and self.welcome_first:
            self.effect('welcome')               # 테이블을 고르자마자 환영 (손님 찾기는 그다음)
        if self.model is None:
            if self.require_detector:
                self.reply(rid, False, 'no_detector')
            else:
                if action == 'greet' and not self.welcome_first:
                    self.effect('welcome')
                self.reply(rid, True, 'no_detector')
            return
        cancel = threading.Event()
        self.job = (rid, action, node, cancel)
        threading.Thread(target=self.watch, args=(rid, action, node, cancel), daemon=True).start()

    def on_cancel(self, msg):
        try:
            rid = json.loads(msg.data).get('id')
        except ValueError:
            return
        if self.job and self.job[0] == rid:
            self.job[3].set()
            self.get_logger().info(f'안내 확인 취소 (id {rid})')

    # ---------------------------------------------------------------- 손님 찾기 (스레드)
    def watch(self, rid, action, node, cancel):
        self.get_logger().info(f'{node}: 손님 {"기다림" if action == "greet" else "확인"} (최대 {self.wait_sec:.0f}초)')
        end = time.monotonic() + self.wait_sec
        period = 1.0 / max(0.5, self.infer_hz)
        hits, last_t, got_frame = 0, 0.0, False
        while not cancel.is_set() and time.monotonic() < end:
            with self.frame_lock:
                msg, ft = self.frame, self.frame_t
            if msg is None or ft == last_t:
                time.sleep(0.05)
                continue
            last_t, got_frame = ft, True
            t0 = time.monotonic()
            try:
                r = self.model(self.to_bgr(msg), verbose=False, conf=self.conf)[0]
                seen = r.boxes is not None and any(int(c) == self.class_id for c in r.boxes.cls.tolist())
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'YOLO 실패: {e}')
                seen = False
            hits = hits + 1 if seen else 0
            if hits >= self.confirm:
                if action == 'greet' and not self.welcome_first:
                    self.effect('welcome')
                self.reply(rid, True, 'guest_seen')
                return
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        if cancel.is_set():
            return                       # 취소: 결과 안 보냄 (fleet_agent가 다시 요청함)
        self.reply(rid, False, 'not_seen' if got_frame else 'no_camera')

    def reply(self, rid, ok, detail):
        self.result_pub.publish(String(data=json.dumps({'id': rid, 'ok': ok, 'detail': detail})))
        self.get_logger().info(f'안내 결과 id {rid}: {"OK" if ok else "못 찾음"} ({detail})')

    def effect(self, kind):
        if self.effects:
            threading.Thread(target=play, args=(kind, self.get_logger().info), daemon=True).start()


def main():
    rclpy.init()
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
