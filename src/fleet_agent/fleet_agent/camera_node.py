"""Camera 노드: 로봇 카메라(picamera2)를 혼자 열고 영상을 토픽으로 발행한다

라즈베리파이 카메라는 한 프로세스만 열 수 있다. 그래서 이 노드만 카메라를 열고, 나머지는 전부 구독한다.
  docking_node · marker_dock · marker_align (마커 도킹)  -> marker_dock.Camera가 구독
  guide_node (안내 손님 YOLO) · detect_view (인식 튜닝)  -> 구독

보냄  /camera/image_raw             sensor_msgs/Image, bgr8 (rotate_180이면 돌려서 = 똑바로 선 영상)
      /camera/image_raw/compressed  sensor_msgs/CompressedImage jpeg (PC guide_view용, 구독자가 있을 때만 압축)

  ros2 run fleet_agent camera_node                                  # 혼자
  fleet_agent.launch.xml / robot_system.launch.xml 은 use_camera:=true (기본)로 같이 띄움

파라미터 (fleet_agent.yaml camera:)
  width, height   : 640 x 480 (marker_dock 보정값 FX가 이 크기 기준. 바꾸면 --calibrate 다시)
  fps             : 15
  rotate_180      : true (카메라가 거꾸로 달림. 예전에 marker_dock이 하던 회전을 여기서 한 번만)
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

TOPIC = '/camera/image_raw'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        p = self.declare_parameter
        self.width = int(p('width', 640).value)
        self.height = int(p('height', 480).value)
        self.fps = float(p('fps', 15.0).value)
        self.rotate = bool(p('rotate_180', True).value)
        self.topic = p('topic', TOPIC).value
        self.frame_id = p('frame_id', 'camera_link').value
        self.compressed = bool(p('publish_compressed', True).value)   # 보는 쪽(guide_view)이 있을 때만 압축
        self.jpeg_quality = int(p('jpeg_quality', 70).value)

        self.pub = self.create_publisher(Image, self.topic, qos_profile_sensor_data)
        self.cpub = (self.create_publisher(CompressedImage, self.topic + '/compressed', qos_profile_sensor_data)
                     if self.compressed else None)
        self.cam = self.open_camera()
        self.running = True
        self.count = 0
        self.t_report = time.monotonic()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'카메라 발행 시작 {self.topic} {self.width}x{self.height} {self.fps:.0f}fps'
                               f'{" (180도 돌림)" if self.rotate else ""}')

    def open_camera(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        # RGB888 = 메모리 순서 B,G,R (OpenCV와 같음) -> bgr8 그대로
        # marker_dock이 쓰던 것과 같은 preview 설정 (설정이 다르면 센서 모드 · 화각이 달라져 도킹 보정 FX가 틀어짐)
        cam.configure(cam.create_preview_configuration(
            main={'format': 'RGB888', 'size': (self.width, self.height)}))
        cam.start()
        time.sleep(1.0)
        return cam

    def loop(self):
        period = 1.0 / max(1.0, self.fps)
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            try:
                frame = self.cam.capture_array()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'카메라 읽기 실패: {e}')
                time.sleep(0.5)
                continue
            if self.rotate:
                frame = np.ascontiguousarray(frame[::-1, ::-1])
            self.publish(frame)
            self.count += 1
            if t0 - self.t_report > 30.0:
                self.get_logger().info(f'{self.count / (t0 - self.t_report):.1f} fps 발행 중')
                self.count, self.t_report = 0, t0
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def publish(self, frame):
        stamp = self.get_clock().now().to_msg()
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.height, m.width = frame.shape[:2]
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = m.width * 3
        m.data = frame[:, :, :3].tobytes()
        self.pub.publish(m)
        if self.cpub is not None and self.cpub.get_subscription_count() > 0:
            import cv2
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                c = CompressedImage()
                c.header.stamp = stamp
                c.header.frame_id = self.frame_id
                c.format = 'jpeg'
                c.data = buf.tobytes()
                self.cpub.publish(c)

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()                 # 카메라를 놓아야 다음에 다시 열 수 있다
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
