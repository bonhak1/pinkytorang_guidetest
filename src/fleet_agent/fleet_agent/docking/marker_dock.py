#!/usr/bin/env python3
"""정밀 도킹 v4.3 (Pinky Pro, AprilTag 36h11, 마커 25 mm)

v4.3 (fleet_agent docking 노드에 넣기)
  - main() 안에 있던 이동 · 측정 함수와 도킹 순서를 Docker 클래스로 옮김 (동작 · 상수는 v4.2 그대로)
  - 혼자 실행(아래 명령)과 docking 노드 안 실행 둘 다 같은 Docker를 쓴다
    도킹 노드에서는 노드 executor가 TF를 받으므로 spin=False, 취소되면 Cancelled로 바로 멈춤
  - Camera: 마커 ID를 도킹마다 바꿀 수 있게 (cam.id). 영상은 camera_node의 /camera/image_raw를 구독
            (카메라를 안내 guide_node와 같이 쓰려고. 예전처럼 직접 열기 = --direct)
  - find_marker(): 마커가 안 보이면 제자리에서 30도씩 돌며 찾기 (align: N14 마커 5 -> 6)

핵심: "마커는 멈춰서 정확히 재고, 움직이는 건 바퀴 주행거리(odom)로 한다"
  v3까지는 움직이는 내내 마커를 봐야 해서, 가까이 붙으면 마커가 화면 밖으로 잘려 실패했고
  카메라 지연 때문에 좌우로 출렁였다. 실무 도킹(Nav2 opennav_docking 등)처럼
  도크 위치를 odom 좌표에 찍어 두고, 그 지점으로 odom을 보며 이동한다.

v4.1
  - 멀리(22 cm 이상)서는 마커가 작아 각도를 믿을 수 없다 (30 cm에서 세로변 1 px 차이 = 16도).
    그래서 멀면 마커 "위치"만 보고 먼저 17 cm 앞까지 다가간 뒤, 거기서 각도까지 잰다.
  - 마커 모서리를 서브픽셀로 다듬어 각도 정밀도를 높였다.
  - 끝날 때 "terminate called ... Aborted"가 나던 문제 수정 (백그라운드 스레드 제거).

v4.2 (빠르게, 정밀도 유지)
  - 측정 장수 20 -> 8~12장, 멈춘 뒤 대기 0.3~0.4 -> 0.15~0.2초
  - 첫 판단에서 이미 정면선 근처(1.5 cm, 5도 이내)면 정렬점을 건너뛰고 바로 근접 지점으로
  - 정렬점 재정렬 기준 완화 (8 mm/3도 -> 15 mm/5도). 근접 측정이 어차피 더 정확하게 다시 잰다
  - 목표가 거의 정면(4도 이내)이면 "바라보기 회전"을 생략하고 바로 직진
  - 먼 구간 직진 최대 5 -> 8 cm/s, 회전 최대 0.6 -> 0.8 rad/s, 목표 근처 감속 구간 단축
  - 지연만큼 미리 멈춤(STOP_LEAD_S): 빨라져도 목표를 넘어가지 않게
  - 로그 줄마다 시작 후 경과 시간 표시

동작 순서
  0. [다가가기] 마커가 22 cm보다 멀면, 마커 위치만 보고 17 cm 앞까지 odom으로 다가감
  1. [측정] 멈춰서 마커를 여러 번 재서 도크(마커) 위치·방향을 odom 좌표로 계산
  2. [정렬점] 도크 정면 정렬점까지 odom으로 회전 -> 직진 -> 회전 (가까우면 먼저 후진)
  3. [재측정] 정렬점에서 다시 재서 도크 위치를 보정 (가장 정확한 측정). 어긋남이 크면 2~3 한 번 더
  4. [근접 측정] 도크 정면 11 cm에서 한 번 더 잼 (마커가 커서 각도가 가장 정확)
  5. [진입] 목표 자세까지 odom으로 직진 (마커가 잘려도 상관없음)
  6. [마무리] 도착 후 마커가 보이면 방향을 재서 제자리에서 미세 조정, 최종 상태 기록

각도는 마커 사다리꼴 모양(왼쪽/오른쪽 세로변 길이 차)으로 잰다 (v3과 같음).
odom은 bringup이 내보내는 TF odom -> base_footprint 를 읽는다.

실행 (로봇에서, 다른 터미널에서 bringup 켜기)
  0. 카메라 거리 보정 (처음 한 번, 마커 정면 25~35 cm에 두고):
       ros2 run fleet_agent marker_dock --calibrate
  1. 목표 저장 (로봇을 원하는 최종 위치에 반듯하게 두고, 마커마다 한 번):
       ros2 run fleet_agent marker_dock --record --id 3
  2. 계획만 보기:  ros2 run fleet_agent marker_dock --dry-run --id 3
  3. 실제 도킹:   ros2 run fleet_agent marker_dock --id 3
  (fleet_agent를 켜 두면 docking 노드가 같은 목표 파일로 자동 도킹한다. 이때 카메라는 docking 노드가 쥐고 있으니
   혼자 실행은 fleet_agent를 끄고 한다)
  옵션: --id 0 처럼 마커 ID 지정 (기본 2)
로그: ~/dock_logs/<시각>_<모드>_v4/
중간에 멈추기: Ctrl+C (로봇 즉시 정지)
"""
import argparse
import datetime
import json
import math
import os
import time

import cv2
import numpy as np

# ---- 마커 / 카메라 ----
MARKER_SIZE_M = 0.025
WIDTH, HEIGHT = 640, 480
FX_DEFAULT = 500.0          # --calibrate 결과가 있으면 그 값을 쓴다
ROTATE_180 = True
CAM_X_M = 0.035             # URDF: base_link -> 카메라 앞쪽 0.020 + 0.015
CAM_FILE = os.path.expanduser('~/dock_cam.json')

# ---- 측정 ----
MEASURE_FRAMES = 10         # 멈춘 상태에서는 매 장이 거의 같아서 많이 찍을 필요 없음 (v4.1 로그: 흔들림 0.1~0.5도)
FAR_MEASURE_FRAMES = 12     # 처음(먼 거리) 측정
SETTLE_S = 0.15             # 멈춘 뒤 대기
MEASURE_SETTLE_S = 0.2      # 측정 전 대기
FACE_SKIP_DEG = 4.0         # 목표 방향이 이보다 작게 틀어져 있으면 바라보기 회전 생략
PRE_APPROACH_Z_M = 0.22     # 마커가 이보다 멀면 각도는 믿지 않고 먼저 다가간다
PRE_TARGET_Z_M = 0.17       # 다가가서 멈출 카메라-마커 거리
STAGE_DIST_M = 0.16         # 정렬점: 도크(마커)에서 로봇 중심까지 정면 거리 (카메라는 약 12.5 cm)
REALIGN_LAT_M = 0.015       # 정면선에서 이보다 벗어나 있으면
REALIGN_YAW_DEG = 5.0       # 또는 이보다 틀어져 있으면 정렬점으로 (근접 측정이 나머지를 고침)
MAX_STAGE_PASSES = 2
MAX_BACKUP_M = 0.15
CLOSE_DIST_M = 0.11        # 진입 전 한 번 더 재는 지점 (로봇 중심, 카메라는 약 7.5 cm) - 마커가 커서 각도가 정확
FINAL_TOL_DEG = 0.7        # 도착 후 카메라로 잰 방향 오차가 이보다 크면 제자리에서 고침
FIND_STEP_DEG = 30.0       # (v4.3) 마커 찾기: 이만큼씩 돌며 본다
FIND_FRAMES = 3

# ---- odom 이동 ----
ROT_W_MAX = 0.8             # rad/s
ROT_W_MIN = 0.15            # 이보다 작으면 바퀴가 안 돎 (v2.1 로그)
ROT_K = 2.0
ROT_TOL_DEG = 0.8
LIN_V_MAX = 0.08            # m/s
LIN_V_MIN = 0.02
LIN_K = 2.0
STOP_LEAD_S = 0.06          # 명령이 실제로 멈추기까지의 지연만큼 미리 멈춤 (넘어감 방지)
LIN_TOL_M = 0.002
HEADING_K = 2.0             # 직진 중 방향 유지
MOVE_HZ = 30.0
MOVE_TIMEOUT_S = 15.0

K = None
DIST = np.zeros(5)
_s = MARKER_SIZE_M / 2
OBJ = np.array([[-_s, _s, 0], [_s, _s, 0], [_s, -_s, 0], [-_s, -_s, 0]], dtype=np.float64)
FX = FX_DEFAULT

LOG_DIR = None
_log_file = None
_T0 = time.time()


def set_fx(fx):
    global FX, K
    FX = fx
    K = np.array([[fx, 0, WIDTH / 2], [0, fx, HEIGHT / 2], [0, 0, 1]], dtype=np.float64)


set_fx(FX_DEFAULT)


def load_cam_file():
    """(v4.3) ~/dock_cam.json의 FX 읽기. 없으면 기본값"""
    if os.path.exists(CAM_FILE):
        with open(CAM_FILE) as f:
            set_fx(json.load(f)['fx'])


def target_file_of(marker_id):
    return os.path.expanduser(f'~/dock_target_id{marker_id}.json')


def open_log(tag):
    """(v4.3) 로그 폴더 새로 열기 (도킹 노드는 도킹마다 하나)"""
    global LOG_DIR, _log_file, _T0
    if _log_file:
        _log_file.close()
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    LOG_DIR = os.path.expanduser(f'~/dock_logs/{stamp}_{tag}_v4')
    os.makedirs(LOG_DIR, exist_ok=True)
    _log_file = open(os.path.join(LOG_DIR, 'run.log'), 'w', encoding='utf-8')
    _T0 = time.time()


def close_log():
    global _log_file
    if _log_file:
        _log_file.close()
        _log_file = None


def log(msg=''):
    msg = f'{time.time() - _T0:5.1f}s {msg}' if msg else msg
    print(msg)
    if _log_file:
        _log_file.write(msg + '\n')
        _log_file.flush()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


# ---------------------------------------------------------------- 측정 (카메라 없이 시험 가능한 부분)

def measure_corners(c):
    """마커 네 모서리(px) -> tx, tz(카메라 기준 m), yaw(벽을 똑바로 보려면 돌 각, + 왼쪽)"""
    c = np.asarray(c, dtype=np.float64).reshape(4, 2)
    ok, _, tvec = cv2.solvePnP(OBJ, c, K, DIST, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    tx, _, tz = tvec.flatten()
    o = c[np.argsort(c[:, 0])]
    h_left = np.linalg.norm(o[0] - o[1])
    h_right = np.linalg.norm(o[2] - o[3])
    r = (h_left - h_right) / (h_left + h_right)
    yaw = math.asin(clamp(2 * tz * r / MARKER_SIZE_M, -1.0, 1.0))
    cx, cy = c.mean(axis=0)
    return {'tx': float(tx), 'tz': float(tz), 'yaw': yaw, 'cx': float(cx), 'cy': float(cy),
            'h_left': float(h_left), 'h_right': float(h_right)}


def marker_in_robot(m):
    """로봇 좌표(X 앞, Y 왼쪽): 마커 위치 M, 도크 법선 n (벽 -> 로봇 쪽)"""
    M = np.array([CAM_X_M + m['tz'], -m['tx']])
    n = np.array([-math.cos(m['yaw']), -math.sin(m['yaw'])])
    return M, n


def dock_in_odom(m, pose):
    """측정값 + 그때 로봇 자세(odom x, y, th) -> 도크 (위치 D, 법선 방향 각 phi)"""
    M, n = marker_in_robot(m)
    R = rot(pose[2])
    D = np.array(pose[:2]) + R @ M
    no = R @ n
    return D, math.atan2(no[1], no[0])


def goal_from_record(m):
    """목표 저장 때 측정값 -> 도크 좌표계에서 로봇 자세 (앞 거리 gx, 옆 gy, 방향 gth)
    도크 좌표계: 원점 = 마커, x축 = 법선(벽 -> 로봇), y축 = x축에서 왼쪽 90도"""
    D, phi = dock_in_odom(m, (0.0, 0.0, 0.0))     # 로봇 자신을 원점으로 둔 좌표
    rel = rot(-phi) @ (np.zeros(2) - D)
    return {'gx': float(rel[0]), 'gy': float(rel[1]), 'gth': wrap(0.0 - phi)}


def dock_to_odom(D, phi, gx, gy, gth):
    """도크 좌표계의 자세 -> odom 자세"""
    p = np.asarray(D) + rot(phi) @ np.array([gx, gy])
    return float(p[0]), float(p[1]), wrap(phi + gth)


def offset_from_line(pose, D, phi, gy, gth):
    """로봇이 도크 정면선(목표 옆 위치 gy)에서 얼마나 벗어났나: (옆 m, 방향 rad, 앞 거리 m)"""
    rel = rot(-phi) @ (np.array(pose[:2]) - np.asarray(D))
    return float(rel[1] - gy), wrap(pose[2] - (phi + gth)), float(rel[0])


# ---------------------------------------------------------------- 카메라

class Camera:
    """마커 측정용 영상. 기본은 camera_node가 발행하는 /camera/image_raw를 구독 (카메라를 다른 노드와 같이 씀)

    Camera(marker_id, node, spin=False)  : 토픽 구독. node = 구독을 만들 rclpy 노드
        spin=False : 노드 executor가 따로 돈다 (docking_node). 프레임은 콜백이 넣고 여기선 기다리기만
        spin=True  : 혼자 실행 (marker_dock / marker_align 메인). 기다리는 동안 직접 spin_once
    Camera(marker_id)                    : 예전처럼 picamera2를 직접 엶 (camera_node를 끈 상태에서만. --direct)
    토픽 영상은 camera_node가 이미 180도 돌려서 내므로 여기서는 돌리지 않는다 (직접 열 때만 ROTATE_180)
    """
    FRAME_TIMEOUT = 1.5      # 새 프레임을 이만큼 못 받으면 카메라 문제로 봄

    def __init__(self, marker_id, node=None, spin=False, topic='/camera/image_raw'):
        self.id = marker_id   # (v4.3) 도킹마다 바꿔도 됨
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # 모서리 서브픽셀
        params.cornerRefinementWinSize = 5
        self.det = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
        self.last_frame = None
        self.node = node
        self.spin = spin
        self.cam = None
        if node is None:                       # 예전 방식: 직접 엶
            from picamera2 import Picamera2
            self.cam = Picamera2()
            self.cam.configure(self.cam.create_preview_configuration(
                main={'format': 'RGB888', 'size': (WIDTH, HEIGHT)}))
            self.cam.start()
            time.sleep(1.0)
            return
        import threading
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._msg = None
        self._seq = 0            # 받은 프레임 번호
        self._used = 0           # 마지막으로 쓴 프레임 번호
        self.topic = topic
        self.sub = node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        with self._lock:
            self._msg = msg
            self._seq += 1
        self._new.set()

    def capture(self):
        """새 프레임 하나 (BGR). 직전에 쓴 것보다 새 것을 기다린다 (예전 capture_array처럼, 움직인 뒤의 영상)"""
        if self.cam is not None:
            frame = self.cam.capture_array()
            return cv2.rotate(frame, cv2.ROTATE_180) if ROTATE_180 else frame
        end = time.monotonic() + self.FRAME_TIMEOUT
        while True:
            with self._lock:
                msg, seq = self._msg, self._seq
                if seq > self._used:
                    self._used = seq
                    self._new.clear()
                    return image_to_bgr(msg)
            left = end - time.monotonic()
            if left <= 0:
                raise RuntimeError(f'카메라 영상 없음 ({self.topic}): camera_node가 떠 있는지 확인')
            if self.spin:
                import rclpy
                rclpy.spin_once(self.node, timeout_sec=min(0.05, left))
            else:
                self._new.wait(min(0.05, left))

    def measure(self):
        frame = self.capture()
        self.last_frame = frame
        corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if i == self.id:
                m = measure_corners(c[0])
                if m:
                    self.last_frame = frame.copy()
                    cv2.aruco.drawDetectedMarkers(self.last_frame, [c], np.array([[i]]))
                return m
        return None

    def measure_stable(self, frames=MEASURE_FRAMES, timeout=4.0):
        got, t0 = [], time.time()
        while len(got) < frames and time.time() - t0 < timeout:
            m = self.measure()
            if m:
                got.append(m)
        if len(got) < frames // 2:
            return None
        out = {k: float(np.median([g[k] for g in got])) for k in got[0]}
        out['yaw_spread'] = float(np.std([g['yaw'] for g in got]))
        out['n'] = len(got)
        return out

    def snapshot(self, name):
        if LOG_DIR and self.last_frame is not None:
            cv2.imwrite(os.path.join(LOG_DIR, name + '.jpg'), self.last_frame)

    def close(self):
        if self.cam is not None:
            self.cam.stop()
            self.cam.close()
        elif self.node is not None:
            try:
                self.node.destroy_subscription(self.sub)
            except Exception:  # noqa: BLE001
                pass


def image_to_bgr(msg):
    """sensor_msgs/Image (bgr8 / rgb8) -> BGR numpy"""
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()


# ---------------------------------------------------------------- 이동 · 도킹 (v4.2 main 안의 함수들)

class Cancelled(Exception):
    """(v4.3) docking 노드가 취소함 -> 주행 함수 안에서 바로 멈춘다"""


class Docker:
    """odom 이동 + 마커 측정 + 도킹 순서

    node, pub, tf_buf : rclpy 노드, /cmd_vel 퍼블리셔, TF 버퍼 (만든 쪽이 넘겨줌)
    cam               : Camera
    moving            : False면 바퀴를 안 움직임 (--dry-run, --record)
    spin              : True = 혼자 실행이라 TF를 직접 받음 / False = docking 노드 executor가 받음
    cancelled         : 호출하면 True/False (docking 노드 취소용)
    """

    def __init__(self, node, pub, tf_buf, cam, moving=True, calibrate=False, spin=True, cancelled=None):
        self.node = node
        self.pub = pub
        self.tf_buf = tf_buf
        self.cam = cam
        self.moving = moving
        self.calibrate_mode = calibrate
        self.spin = spin
        self.cancelled = cancelled or (lambda: False)

    def pose(self):
        from rclpy.time import Time
        if self.spin:
            import rclpy
            for _ in range(20):                       # 쌓인 TF 메시지 처리 (별도 스레드 없이)
                rclpy.spin_once(self.node, timeout_sec=0.0)
        t = self.tf_buf.lookup_transform('odom', 'base_footprint', Time()).transform
        q = t.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.translation.x, t.translation.y, th)

    def wait_pose(self, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                return self.pose()
            except Exception:
                if self.spin:
                    import rclpy
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                else:
                    time.sleep(0.1)
        raise RuntimeError('odom TF(odom -> base_footprint)를 받을 수 없음. bringup이 켜져 있나요?')

    def send(self, v, w):
        if self.cancelled():
            from geometry_msgs.msg import Twist
            self.pub.publish(Twist())
            raise Cancelled()
        if self.moving or self.calibrate_mode:
            from geometry_msgs.msg import Twist
            t = Twist()
            t.linear.x = float(v)
            t.angular.z = float(w)
            self.pub.publish(t)

    def stop(self, settle=SETTLE_S):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.03)
        time.sleep(settle)

    def rotate_to(self, target):
        t0 = time.time()
        w_cmd = 0.0
        while time.time() - t0 < MOVE_TIMEOUT_S:
            err = wrap(target - self.pose()[2])
            if abs(err) < max(math.radians(ROT_TOL_DEG), abs(w_cmd) * STOP_LEAD_S):
                break
            w = clamp(ROT_K * err, -ROT_W_MAX, ROT_W_MAX)
            w_cmd = math.copysign(max(abs(w), ROT_W_MIN), w)
            self.send(0.0, w_cmd)
            time.sleep(1 / MOVE_HZ)
        self.stop()
        return math.degrees(wrap(target - self.pose()[2]))

    def drive_to(self, x, y, reverse=False):
        """(x, y)까지 직진. 출발할 때의 직선 방향을 유지"""
        p0 = self.pose()
        line = math.atan2(y - p0[1], x - p0[0])
        hold = wrap(line + math.pi) if reverse else line
        u = np.array([math.cos(line), math.sin(line)])
        t0 = time.time()
        v = 0.0
        while time.time() - t0 < MOVE_TIMEOUT_S:
            p = self.pose()
            remain = float(u @ (np.array([x, y]) - np.array(p[:2])))
            if remain < max(LIN_TOL_M, v * STOP_LEAD_S):
                break
            v = clamp(LIN_K * remain, LIN_V_MIN, LIN_V_MAX)
            w = clamp(HEADING_K * wrap(hold - p[2]), -0.4, 0.4)
            self.send(-v if reverse else v, w)
            time.sleep(1 / MOVE_HZ)
        self.stop()
        p = self.pose()
        return float(u @ (np.array([x, y]) - np.array(p[:2])))

    def back_off(self, dist):
        """(v4.3) 지금 방향 그대로 dist(m) 후진 (precision_stops의 back)"""
        p = self.pose()
        return self.drive_to(p[0] - dist * math.cos(p[2]), p[1] - dist * math.sin(p[2]), reverse=True)

    def move_pose(self, x, y, th, tag):
        """odom 자세 (x, y, th)로: 바라보기 -> 직진 -> 방향 맞추기"""
        p = self.pose()
        dist = math.hypot(x - p[0], y - p[1])
        if dist > 0.004:
            face = math.atan2(y - p[1], x - p[0])
            if abs(math.degrees(wrap(face - p[2]))) > FACE_SKIP_DEG:
                e1 = self.rotate_to(face)
            else:
                e1 = math.degrees(wrap(face - p[2]))      # 거의 정면이라 회전 없이 직진하며 바로잡음
            left = self.drive_to(x, y)
            log(f'[{tag}] 바라보기 오차 {e1:+.1f}°, 직진 {dist * 100:.1f} cm (남은 {left * 1000:+.1f} mm)')
        e2 = self.rotate_to(th)
        log(f'[{tag}] 방향 맞추기 오차 {e2:+.1f}°')

    def measure_dock(self, tag, frames=MEASURE_FRAMES):
        self.stop(MEASURE_SETTLE_S)
        m = self.cam.measure_stable(frames=frames)
        self.cam.snapshot(tag)
        if m is None:
            return None
        p = self.wait_pose()
        D, phi = dock_in_odom(m, p)
        log(f'[{tag}] 마커 앞 {m["tz"] * 100:.1f} cm, 좌우 {m["tx"] * 100:+.1f} cm, '
            f'각도 {math.degrees(m["yaw"]):+.1f}° (흔들림 {math.degrees(m["yaw_spread"]):.1f}°, {m["n"]}회) '
            f'| 세로변 왼 {m["h_left"]:.0f} / 오 {m["h_right"]:.0f}')
        log(f'[{tag}] 로봇 odom ({p[0]:.3f}, {p[1]:.3f}, {math.degrees(p[2]):+.1f}°) '
            f'-> 도크 odom ({D[0]:.3f}, {D[1]:.3f}), 방향 {math.degrees(phi):+.1f}°')
        return m, p, D, phi

    def find_marker(self):
        """(v4.3) 마커가 안 보이면 제자리에서 FIND_STEP_DEG씩 돌며 찾기 (한 바퀴까지)"""
        for k in range(int(360 / FIND_STEP_DEG)):
            self.stop(MEASURE_SETTLE_S)
            if self.cam.measure_stable(frames=FIND_FRAMES, timeout=1.0) is not None:
                if k:
                    log(f'[찾기] 마커 {self.cam.id}를 {k * FIND_STEP_DEG:.0f}° 돌아서 찾음')
                return True
            self.rotate_to(wrap(self.pose()[2] - math.radians(FIND_STEP_DEG)))
        log(f'[찾기] 한 바퀴 돌아도 마커 {self.cam.id}가 안 보임')
        return False

    # ------------------------------------------------------------ 모드
    def calibrate(self):
        """카메라 거리 보정 -> 결과 문구"""
        self.stop(0.3)
        m1 = self.cam.measure_stable()
        if m1 is None:
            return '실패: 마커가 안 보임'
        p1 = self.pose()
        self.drive_to(p1[0] + 0.10 * math.cos(p1[2]), p1[1] + 0.10 * math.sin(p1[2]))
        self.stop(0.4)
        m2 = self.cam.measure_stable()
        p2 = self.pose()
        if m2 is None:
            return '실패: 이동 후 마커가 안 보임 (더 멀리서 시작)'
        d_odom = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        d_cam = m1['tz'] - m2['tz']
        fx = FX * d_odom / d_cam
        log(f'[보정] odom 이동 {d_odom * 100:.2f} cm, 카메라 거리 변화 {d_cam * 100:.2f} cm '
            f'-> FX {FX:.1f} -> {fx:.1f}')
        if not 300 < fx < 900:
            return f'실패: 보정값 {fx:.0f}이 이상함 (저장 안 함)'
        with open(CAM_FILE, 'w') as f:
            json.dump({'fx': fx}, f, indent=2)
        return f'보정 저장: FX {fx:.1f} -> {CAM_FILE}. 이후 --record를 다시 하세요'

    def record(self, target_file):
        """지금 위치를 도킹 목표로 저장 -> 결과 문구"""
        m = self.cam.measure_stable(frames=30)
        self.cam.snapshot('record')
        if m is None:
            return '실패: 마커가 안 보임 (목표 저장 안 됨)'
        g = goal_from_record(m)
        g['fx'] = FX
        with open(target_file, 'w') as f:
            json.dump(g, f, indent=2)
        log(f'[목표] 마커 앞 {m["tz"] * 100:.1f} cm, 좌우 {m["tx"] * 100:+.1f} cm, 각도 {math.degrees(m["yaw"]):+.1f}°')
        return (f'목표 저장: 도크에서 앞 {g["gx"] * 100:.1f} cm, 옆 {g["gy"] * 100:+.1f} cm, '
                f'방향 {math.degrees(g["gth"]):+.1f}° -> {target_file}')

    def dock(self, target_file, dry_run=False):
        """도킹 (0~6단계) -> 결과 문구. '성공'이면 성공"""
        if not os.path.exists(target_file):
            return f'실패: 목표 파일 없음. 먼저 최종 위치에서 --record ({target_file})'
        with open(target_file) as f:
            g = json.load(f)
        if 'gx' not in g:
            return '실패: 이전 버전 목표 파일입니다. v4에서 --record를 다시 해 주세요'
        if abs(g.get('fx', FX) - FX) > 1:
            log(f'주의: 목표 저장 때 FX {g.get("fx")}와 지금 FX {FX:.1f}가 다릅니다. --record를 다시 하는 것을 권합니다')
        log(f'도킹 시작 [{"DRY-RUN" if dry_run else "실제 주행"}] 목표: 도크 앞 {g["gx"] * 100:.1f} cm, '
            f'옆 {g["gy"] * 100:+.1f} cm, 방향 {math.degrees(g["gth"]):+.1f}°')

        # ---------- 1. 측정 ----------
        r = self.measure_dock('측정', FAR_MEASURE_FRAMES)
        if r is None:
            return '실패: 마커가 안 보임'
        m, p, D, phi = r

        # ---------- 0. 멀면 먼저 다가가기 (위치만 사용) ----------
        if m['tz'] > PRE_APPROACH_Z_M:
            Mo = np.array(p[:2]) + rot(p[2]) @ np.array([CAM_X_M + m['tz'], -m['tx']])
            u = Mo - np.array(p[:2])
            u = u / np.linalg.norm(u)
            tgt = Mo - u * (PRE_TARGET_Z_M + CAM_X_M)
            face = math.atan2(u[1], u[0])
            log(f'[다가가기] 마커가 {m["tz"] * 100:.1f} cm로 멀어서 각도는 쓰지 않고, '
                f'마커 쪽으로 {(m["tz"] - PRE_TARGET_Z_M) * 100:.1f} cm 다가감')
            if dry_run:
                return 'DRY-RUN 끝 (다가간 뒤의 계획은 실제로 다가가야 알 수 있음)'
            self.move_pose(tgt[0], tgt[1], face, tag='다가가기')
            r = self.measure_dock('가까이 측정')
            if r is None:
                return '실패: 다가간 뒤 마커가 안 보임'
            m, p, D, phi = r

        for k in range(1, MAX_STAGE_PASSES + 1):
            lat, dth, along = offset_from_line(p, D, phi, g['gy'], g['gth'])
            log(f'[판단 {k}] 정면선에서 옆 {lat * 100:+.1f} cm, 방향 {math.degrees(dth):+.1f}°, 도크까지 {along * 100:.1f} cm')
            if abs(lat) < REALIGN_LAT_M and abs(math.degrees(dth)) < REALIGN_YAW_DEG \
                    and along > CLOSE_DIST_M + 0.02:
                log('  -> 정면선 근처라 정렬점 생략, 바로 근접 지점으로' if k == 1 else '  -> 정면선 위에 있음')
                break
            # ---------- 2. 정렬점 ----------
            stage = dock_to_odom(D, phi, STAGE_DIST_M, g['gy'], g['gth'])
            need_back = (STAGE_DIST_M + 0.02 - along) if along < STAGE_DIST_M - 0.005 else 0.0
            log(f'  -> 정렬점 odom ({stage[0]:.3f}, {stage[1]:.3f}, {math.degrees(stage[2]):+.1f}°)'
                + (f', 먼저 {need_back * 100:.1f} cm 후진' if need_back > 0 else ''))
            if dry_run:
                return 'DRY-RUN 끝 (계획만 출력)'
            if need_back > 0:
                back = min(need_back, MAX_BACKUP_M)
                self.drive_to(p[0] - back * math.cos(p[2]), p[1] - back * math.sin(p[2]), reverse=True)
            self.move_pose(*stage, tag=f'정렬점 {k}')
            # ---------- 3. 재측정 ----------
            r = self.measure_dock(f'재측정 {k}')
            if r is None:
                log('  -> 정렬점에서 마커가 안 보임, 처음 측정값으로 진행')
                p = self.pose()
                break
            m, p, D, phi = r

        # ---------- 4. 근접 측정 ----------
        close = dock_to_odom(D, phi, CLOSE_DIST_M, g['gy'], g['gth'])
        self.move_pose(*close, tag='근접')
        r = self.measure_dock('근접 측정')
        if r is not None:
            m, p, D, phi = r
        else:
            log('[근접 측정] 마커가 안 보임, 이전 측정값으로 진입')

        # ---------- 5. 진입 ----------
        goal = dock_to_odom(D, phi, g['gx'], g['gy'], g['gth'])
        p = self.pose()
        log(f'[진입] 목표 odom ({goal[0]:.3f}, {goal[1]:.3f}, {math.degrees(goal[2]):+.1f}°), '
            f'거리 {math.hypot(goal[0] - p[0], goal[1] - p[1]) * 100:.1f} cm')
        self.move_pose(*goal, tag='진입')
        p = self.pose()
        lat, dth, along = offset_from_line(p, D, phi, g['gy'], g['gth'])
        log(f'[진입 끝] odom 기준: 정면선에서 옆 {lat * 1000:+.1f} mm, 방향 {math.degrees(dth):+.2f}°, '
            f'앞뒤 {(along - g["gx"]) * 1000:+.1f} mm')

        # ---------- 6. 마무리: 카메라로 방향 미세 조정 ----------
        for k in range(2):
            self.stop(MEASURE_SETTLE_S)
            s2 = self.cam.measure_stable(frames=8, timeout=2.0)
            if s2 is None:
                log('[마무리] 마커가 화면에 다 안 들어와 생략')
                break
            err = wrap(g['gth'] - goal_from_record(s2)['gth'])
            log(f'[마무리 {k + 1}] 카메라로 잰 방향 오차 {math.degrees(err):+.2f}°')
            if abs(math.degrees(err)) < FINAL_TOL_DEG:
                break
            self.rotate_to(wrap(self.pose()[2] + err))

        # 최종 확인 (기록만)
        self.stop(0.2)
        m = self.cam.measure_stable(frames=10, timeout=2.0)
        self.cam.snapshot('final')
        if m:
            g2 = goal_from_record(m)
            log(f'[최종 카메라] 도크 앞 {g2["gx"] * 100:.1f} cm (목표 {g["gx"] * 100:.1f}), '
                f'옆 {g2["gy"] * 100:+.1f} cm (목표 {g["gy"] * 100:+.1f}), '
                f'방향 {math.degrees(g2["gth"]):+.1f}° (목표 {math.degrees(g["gth"]):+.1f}°)')
        else:
            log('[최종 카메라] 마커가 화면에 다 안 들어와 확인 생략 (정상일 수 있음)')
        return '성공'


# ---------------------------------------------------------------- 혼자 실행

def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.signals import SignalHandlerOptions
    from tf2_ros import Buffer, TransformListener

    ap = argparse.ArgumentParser()
    ap.add_argument('--id', type=int, default=2, help='마커 ID (기본 2)')
    ap.add_argument('--name', default=None, help='목표 이름 (같은 마커로 목표 여러 개, 예: n15 언도킹 자리) -> ~/dock_target_<name>.json')
    ap.add_argument('--record', action='store_true', help='현재 위치를 도킹 목표로 저장')
    ap.add_argument('--calibrate', action='store_true', help='odom으로 카메라 거리 보정 (마커 정면 25~35 cm)')
    ap.add_argument('--dry-run', action='store_true', help='바퀴를 움직이지 않고 계획만 출력')
    ap.add_argument('--direct', action='store_true',
                    help='camera_node 없이 카메라를 직접 엶 (camera_node · docking 노드가 꺼져 있어야 함)')
    args, _ = ap.parse_known_args()   # ros2 run 이 붙이는 --ros-args는 무시
    target_file = os.path.expanduser(f'~/dock_target_{args.name}.json') if args.name else target_file_of(args.id)

    tag = 'record' if args.record else 'calib' if args.calibrate else 'dry' if args.dry_run else 'run'
    open_log(tag)

    if not args.calibrate:
        load_cam_file()
    log(f'로그 폴더: {LOG_DIR}')
    log(f'설정: FX {FX:.1f}, CAM_X {CAM_X_M}, STAGE {STAGE_DIST_M}, ROT_W_MIN {ROT_W_MIN}')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('marker_dock')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buf = Buffer()
    tf_listener = TransformListener(tf_buf, node)   # noqa: F841 (살아 있어야 TF를 받음)
    cam = Camera(args.id) if args.direct else Camera(args.id, node, spin=True)   # 기본: camera_node 구독
    dk = Docker(node, pub, tf_buf, cam, moving=not (args.dry_run or args.record), calibrate=args.calibrate, spin=True)

    result = None
    t_start = time.time()
    try:
        dk.wait_pose()
        if args.calibrate:
            result = dk.calibrate()
        elif args.record:
            result = dk.record(target_file)
        else:
            result = dk.dock(target_file, dry_run=args.dry_run)
    except KeyboardInterrupt:
        result = '중단: Ctrl+C'
    except Exception as e:
        result = f'실패: {e}'
    finally:
        try:
            dk.stop(0.2)
        except Exception:
            pass
        cam.close()
        log(f'결과: {result}  ({time.time() - t_start:.1f}초)')
        close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
