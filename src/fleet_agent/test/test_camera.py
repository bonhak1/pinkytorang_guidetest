"""카메라 공유 (camera_node 발행 -> marker_dock.Camera 구독) 시험, ROS 없이:  python3 -m pytest test/test_camera.py

marker_dock.Camera(marker_id, node, spin)이 토픽 영상으로 예전 picamera2 직접 읽기와 똑같이 동작하는지 본다
  - 매번 새 프레임 (움직인 뒤 영상), 영상이 끊기면 분명한 에러
  - bgr8 / rgb8, 행 끝 여백(step)
  - 토픽 영상은 이미 돌려져 있어 다시 돌리지 않음
  - 실제 AprilTag 마커를 토픽으로 받아 측정됨
"""
import os
import sys
import threading
import time
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ROS가 없는 PC: Camera가 쓰는 만큼만 가짜 모듈
try:
    import rclpy  # noqa: F401
    from sensor_msgs.msg import Image as _RosImage  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False
    rclpy_m = types.ModuleType('rclpy')
    rclpy_m.SPIN = []
    rclpy_m.spin_once = lambda node, timeout_sec=0.0: (rclpy_m.SPIN.append(timeout_sec), node.spin_hook())
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = 'sensor_data'
    sm = types.ModuleType('sensor_msgs')
    smm = types.ModuleType('sensor_msgs.msg')

    class _Image:
        pass
    smm.Image = _Image
    sys.modules.update({'rclpy': rclpy_m, 'rclpy.qos': qos, 'sensor_msgs': sm, 'sensor_msgs.msg': smm})

cv2 = pytest.importorskip('cv2')
from fleet_agent.docking import marker_dock as md  # noqa: E402

pytestmark = pytest.mark.skipif(HAVE_ROS, reason='가짜 rclpy로만 도는 시험 (ROS 있는 로봇에서는 실제로 확인)')


class Msg:
    def __init__(self, bgr, encoding='bgr8', pad=0):
        h, w = bgr.shape[:2]
        img = bgr[:, :, ::-1] if encoding == 'rgb8' else bgr
        rows = np.zeros((h, w * 3 + pad), np.uint8)
        rows[:, :w * 3] = img.reshape(h, w * 3)
        self.height, self.width, self.encoding, self.step = h, w, encoding, w * 3 + pad
        self.data = rows.tobytes()


class FakeNode:
    def __init__(self):
        self.cb = None
        self.topic = None
        self.qos = None
        self.pending = []      # spin_once 때 넣을 메시지 (spin=True 시험)

    def create_subscription(self, typ, topic, cb, qos):
        self.cb, self.topic, self.qos = cb, topic, qos
        return 'sub'

    def destroy_subscription(self, sub):
        self.cb = None

    def spin_hook(self):
        if self.pending:
            self.cb(self.pending.pop(0))


def frame(v):
    f = np.zeros((48, 64, 3), np.uint8)
    f[:, :, 0], f[:, :, 1], f[:, :, 2] = v, v + 1, v + 2
    return f


def test_subscribes_camera_topic_best_effort():
    n = FakeNode()
    md.Camera(3, n)
    assert n.topic == '/camera/image_raw' and n.qos == 'sensor_data'


def test_capture_waits_for_a_new_frame_each_time():
    n = FakeNode()
    cam = md.Camera(3, n, spin=False)
    n.cb(Msg(frame(10)))
    assert cam.capture()[0, 0, 0] == 10
    # 같은 프레임을 두 번 주지 않음: 다음 프레임이 올 때까지 기다림
    threading.Timer(0.2, lambda: n.cb(Msg(frame(20)))).start()
    t0 = time.monotonic()
    f = cam.capture()
    assert f[0, 0, 0] == 20 and time.monotonic() - t0 >= 0.15


def test_capture_uses_latest_when_several_arrived():
    n = FakeNode()
    cam = md.Camera(3, n)
    for v in (1, 2, 3):
        n.cb(Msg(frame(v)))
    assert cam.capture()[0, 0, 0] == 3


def test_no_frames_raises_clear_error():
    n = FakeNode()
    cam = md.Camera(3, n)
    cam.FRAME_TIMEOUT = 0.2
    with pytest.raises(RuntimeError, match='camera_node'):
        cam.capture()


def test_rgb8_and_row_padding():
    n = FakeNode()
    cam = md.Camera(3, n)
    n.cb(Msg(frame(50), encoding='rgb8', pad=8))
    f = cam.capture()
    assert f.shape == (48, 64, 3) and list(f[5, 5]) == [50, 51, 52]


def test_topic_frames_are_not_rotated_again():
    n = FakeNode()
    cam = md.Camera(3, n)
    f = np.zeros((48, 64, 3), np.uint8)
    f[0, 0] = 255                      # 왼쪽 위 점
    n.cb(Msg(f))
    assert cam.capture()[0, 0, 0] == 255


def test_spin_mode_spins_node_while_waiting():
    n = FakeNode()
    cam = md.Camera(3, n, spin=True)
    n.pending = [Msg(frame(7))]
    assert cam.capture()[0, 0, 0] == 7
    assert sys.modules['rclpy'].SPIN, '혼자 실행이면 기다리는 동안 spin_once'


def test_close_unsubscribes():
    n = FakeNode()
    cam = md.Camera(3, n)
    cam.close()
    assert n.cb is None


def test_marker_measured_from_topic():
    """실제 AprilTag(36h11) 그림을 토픽으로 넣으면 measure가 값을 낸다"""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(d, 3, 160)
    img = np.full((md.HEIGHT, md.WIDTH), 255, np.uint8)
    y, x = (md.HEIGHT - 160) // 2, (md.WIDTH - 160) // 2
    img[y:y + 160, x:x + 160] = tag
    bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    n = FakeNode()
    cam = md.Camera(3, n)
    n.cb(Msg(bgr))
    m = cam.measure()
    assert m is not None, '마커 3을 못 찾음'
    assert abs(m.get('tx', 0.0)) < 0.01          # 화면 가운데 = 옆 오차 거의 0
    cam.id = 5                                    # 다른 마커 ID면 못 찾음
    n.cb(Msg(bgr))
    assert cam.measure() is None


def test_guide_view_records_both_videos_and_extracts(tmp_path):
    from fleet_agent.guide_view import Painter, Recorder, ViewState, extract_frames
    st = ViewState(0.5, 2, 'toy_person')
    p = Painter()
    rec = Recorder(str(tmp_path), fps=15)
    t = time.monotonic()
    for i in range(30):
        st.frame, st.frame_t = frame(i * 5), t + i      # 새 프레임 30장
        canvas = p.render(st, t + i)
        if i == 0:
            rec.start(canvas, st.frame)
            st.rec_since = t
        rec.write(canvas, st.frame, st.frame_t)
        rec.write(canvas, st.frame, st.frame_t)            # 같은 프레임 두 번 -> 한 번만
    (pv, pr), n = rec.stop()
    assert n == 30 and os.path.getsize(pv) > 0 and os.path.getsize(pr) > 0
    for path, w in ((pv, 64 + 300), (pr, 64)):
        cap = cv2.VideoCapture(path)
        assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 30 and int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) == w
        cap.release()
    saved = extract_frames(pr, str(tmp_path / 'frames'), every=5)
    assert saved == 6 and len(os.listdir(tmp_path / 'frames')) == 6
