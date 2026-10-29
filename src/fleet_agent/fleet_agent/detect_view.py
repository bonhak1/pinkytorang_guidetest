"""손님 인식(YOLO) 튜닝: guide_node와 같은 모델 · 같은 판정(conf, confirm 연속)으로 계속 돌려 보며 숫자를 찍는다

guide_node는 요청이 있을 때만 돌고 OK / 못 찾음만 알려 준다. 이건 쉬지 않고 돌려서
"몇 m까지 · 어떤 각도까지 · 조명에서 conf가 얼마 나오는지"를 보고 conf · confirm · infer_hz를 정할 때 쓴다.

  ros2 run fleet_agent detect_view                               # 로봇 카메라 토픽 (/camera/image_raw)
  ros2 run fleet_agent detect_view --topic /camera/image_raw/compressed --compressed
  ros2 run fleet_agent detect_view --save /tmp/det --every 5     # 5번째마다 박스 그린 사진 저장
  python3 -m fleet_agent.detect_view --image ./photos            # PC: 사진 폴더 (ROS 없이)
  python3 -m fleet_agent.detect_view --webcam 0                  # PC: 웹캠 (ROS 없이)

  옵션 --conf 0.5 --confirm 2 --hz 4 --class toy_person (기본 = fleet_agent.yaml guide와 같음)
  Ctrl+C -> 요약 (인식률, 평균 conf, 추론 시간)
"""
import argparse
import glob
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def default_model():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('fleet_agent'), 'models', 'best.pt')
        if os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    p = os.path.normpath(os.path.join(HERE, '..', 'models', 'best.pt'))
    return p if os.path.exists(p) else ''


class Judge:
    """guide_node.watch와 같은 판정: 연속 confirm 번 보이면 찾음"""

    def __init__(self, model, class_name, conf, confirm):
        self.model = model
        self.conf = conf
        self.confirm = confirm
        names = model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        self.names = dict(items)
        ids = [k for k, v in self.names.items() if v == class_name]
        if not ids:
            raise SystemExit(f'모델에 클래스 "{class_name}" 없음. 있는 것: {list(self.names.values())}')
        self.cid = ids[0]
        self.hits = 0
        self.n = self.seen_n = self.confirm_n = 0
        self.confs = []
        self.ms = []

    def run(self, bgr):
        t0 = time.monotonic()
        r = self.model(bgr, verbose=False, conf=self.conf)[0]
        ms = (time.monotonic() - t0) * 1000
        boxes = []
        if r.boxes is not None:
            for c, s, xyxy in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
                boxes.append((int(c), float(s), [int(v) for v in xyxy]))
        mine = [b for b in boxes if b[0] == self.cid]
        seen = bool(mine)
        self.hits = self.hits + 1 if seen else 0
        self.n += 1
        self.ms.append(ms)
        if seen:
            self.seen_n += 1
            self.confs.append(max(b[1] for b in mine))
        ok = self.hits >= self.confirm
        if ok:
            self.confirm_n += 1
        return boxes, mine, ok, ms

    def line(self, boxes, mine, ok, ms, shape):
        h, w = shape[:2]
        if mine:
            best = max(mine, key=lambda b: b[1])
            x1, y1, x2, y2 = best[2]
            cx = (x1 + x2) / 2 / w - 0.5
            txt = (f'손님 {len(mine)}  conf {best[1]:.2f}  박스 높이 {(y2 - y1) / h * 100:3.0f}%  '
                   f'가로 위치 {cx * 100:+4.0f}%')
        else:
            txt = '손님 없음'
        others = [f'{self.names.get(c, c)}:{s:.2f}' for c, s, _ in boxes if c != self.cid]
        return (f'#{self.n:4d} {ms:5.0f}ms  {txt:46s} 연속 {self.hits}/{self.confirm} '
                f'{"✅ 찾음" if ok else "  "}  {" ".join(others)}')

    def summary(self):
        if not self.n:
            return '프레임 없음'
        avg = sum(self.confs) / len(self.confs) if self.confs else 0.0
        mn = min(self.confs) if self.confs else 0.0
        return (f'프레임 {self.n}, 손님 보임 {self.seen_n} ({self.seen_n / self.n * 100:.0f}%), '
                f'찾음 판정 {self.confirm_n}, conf 평균 {avg:.2f} 최저 {mn:.2f}, '
                f'추론 평균 {sum(self.ms) / len(self.ms):.0f}ms (최대 {1000 / max(1e-3, np.mean(self.ms)):.1f}Hz)')


def draw(bgr, boxes, names, cid, ok, path):
    import cv2
    img = bgr.copy()
    for c, s, (x1, y1, x2, y2) in boxes:
        col = (0, 200, 0) if c == cid else (0, 160, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        cv2.putText(img, f'{names.get(c, c)} {s:.2f}', (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    if ok:
        cv2.putText(img, 'FOUND', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
    cv2.imwrite(path, img)


def to_bgr(msg, compressed):
    if compressed:
        import cv2
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img


def main():
    argv = sys.argv[1:]
    try:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    except Exception:  # noqa: BLE001
        pass
    ap = argparse.ArgumentParser(description='손님 인식(YOLO) 튜닝: 계속 돌리며 conf · 박스 크기 · 연속 판정을 찍음')
    ap.add_argument('--model', default='', help='기본: fleet_agent models/best.pt')
    ap.add_argument('--class', dest='cls', default='toy_person')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--confirm', type=int, default=2)
    ap.add_argument('--hz', type=float, default=4.0, help='추론 주기 (guide infer_hz)')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--compressed', action='store_true')
    ap.add_argument('--image', default='', help='사진 파일 또는 폴더 (ROS 없이)')
    ap.add_argument('--webcam', type=int, default=-1, help='웹캠 번호 (ROS 없이)')
    ap.add_argument('--save', default='', help='박스 그린 사진 저장 폴더')
    ap.add_argument('--every', type=int, default=1, help='--save 때 몇 번째마다 저장')
    ap.add_argument('--quiet', action='store_true', help='줄마다 안 찍고 요약만')
    a = ap.parse_args(argv)

    model_path = a.model or default_model()
    if not model_path or not os.path.exists(model_path):
        raise SystemExit(f'모델 없음: {model_path or "(models/best.pt 못 찾음)"}  --model로 지정')
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit('ultralytics 없음: pip install ultralytics')
    print(f'모델 {model_path}, 클래스 {a.cls}, conf {a.conf}, confirm {a.confirm}, {a.hz}Hz')
    judge = Judge(YOLO(model_path), a.cls, a.conf, a.confirm)
    if a.save:
        os.makedirs(a.save, exist_ok=True)

    def handle(bgr, tag=''):
        boxes, mine, ok, ms = judge.run(bgr)
        if not a.quiet:
            print(judge.line(boxes, mine, ok, ms, bgr.shape) + (f'  {tag}' if tag else ''), flush=True)
        if a.save and judge.n % max(1, a.every) == 0:
            draw(bgr, boxes, judge.names, judge.cid, ok, os.path.join(a.save, f'det_{judge.n:05d}.jpg'))

    try:
        if a.image:
            import cv2
            files = sorted(glob.glob(os.path.join(a.image, '*'))) if os.path.isdir(a.image) else [a.image]
            for f in files:
                img = cv2.imread(f)
                if img is not None:
                    judge.hits = 0          # 사진은 서로 따로
                    handle(img, os.path.basename(f))
        elif a.webcam >= 0:
            import cv2
            cap = cv2.VideoCapture(a.webcam)
            period = 1.0 / max(0.5, a.hz)
            while True:
                t0 = time.monotonic()
                ok, img = cap.read()
                if not ok:
                    print('웹캠 프레임 없음')
                    break
                handle(img)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        else:
            run_ros(a, handle)
    except KeyboardInterrupt:
        pass
    print('\n==== 요약 ====\n' + judge.summary())
    print('팁: 손님이 있는데 "찾음"까지 오래 걸리면 conf를 낮추거나 confirm을 줄이고, '
          '없는데 찾음이 나오면 conf를 올린다 (fleet_agent.yaml guide).')


def run_ros(a, handle):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C -> 요약 찍고 끝
    node = Node('detect_view')
    state = {'msg': None, 't': 0.0, 'last': 0.0}

    def on_img(m):
        state['msg'], state['t'] = m, time.monotonic()

    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(CompressedImage if a.compressed else Image, a.topic, on_img, qos_profile_sensor_data)
    print(f'카메라 {a.topic} 기다리는 중...')
    period = 1.0 / max(0.5, a.hz)
    warned = time.monotonic()
    try:
        while rclpy.ok():
            t0 = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
            if state['msg'] is None or state['t'] == state['last']:
                if time.monotonic() - warned > 5.0:
                    print(f'영상 없음 ({a.topic}). ros2 topic hz {a.topic} 로 확인')
                    warned = time.monotonic()
                continue
            state['last'] = state['t']
            handle(to_bgr(state['msg'], a.compressed))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
