"""안내 카메라 확인 창 (PC): 로봇 카메라 영상 + 손님 인식(YOLO) 박스 + 안내 상태를 한 화면에

로봇과 같은 ROS_DOMAIN_ID인 PC에서 띄운다. 로봇은 camera_node가 떠 있으면 된다 (gfake / greal / frun).
영상은 와이파이 부담이 적은 JPEG(/camera/image_raw/compressed)로 받고, YOLO는 PC에서
로봇 guide_node와 같은 모델 · 같은 판정(conf 넘는 손님이 confirm번 연속 = 찾음)으로 돌린다.

  ros2 run fleet_agent guide_view                      # 기본: 압축 영상, models/best.pt, toy_person
  ros2 run fleet_agent guide_view --raw                # 원본 영상 (유선 · 로봇에서 직접 볼 때)
  ros2 run fleet_agent guide_view --conf 0.4 --hz 8    # 처음 conf · 추론 주기

  ros2 run fleet_agent guide_view --record           # 켜자마자 녹화
  python3 -m fleet_agent.guide_view --extract ~/guide_view/raw_172301.mp4 --every 5   # 원본 영상 -> 학습용 사진

  창에서:  [ / ] conf 내리기 · 올리기   s 사진 저장   r 녹화 시작/끝   p 멈춤/계속   q · Esc 끝
  녹화 = ~/guide_view/ 에 두 개:  view_<시각>.mp4 (박스 · 판 그린 화면)  raw_<시각>.mp4 (카메라 원본, 학습 데이터용)
  오른쪽 판:  찾음 판정 · 연속 횟수 · 가장 큰 손님 박스(거리 짐작) · 로봇 안내 요청/결과(guide_node)
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

W_PANEL = 300
KO_FONTS = ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
            '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
            '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
            '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc')

GREEN, ORANGE, RED, GRAY = (60, 200, 60), (0, 160, 255), (60, 60, 230), (150, 150, 150)   # BGR


# ------------------------------------------------------------------ 상태
class ViewState:
    """창에 보여 줄 것 (ROS 콜백 · YOLO 스레드가 채우고, 그리기는 읽기만)"""

    def __init__(self, conf, confirm, class_name):
        self.lock = threading.Lock()
        self.conf = conf
        self.confirm = confirm
        self.class_name = class_name
        self.frame = None            # 최신 영상 (BGR)
        self.frame_t = 0.0
        self.rx_times = []           # 받은 시각 (fps 계산)
        self.boxes = []              # [(cls_name, conf, (x1,y1,x2,y2), 손님인가)]
        self.boxes_t = 0.0
        self.hits = 0
        self.found = False
        self.infer_ms = 0.0
        self.guide_req = None        # (action, node, t)
        self.guide_res = None        # (ok, detail, t)
        self.paused = False
        self.note = ''               # 잠깐 띄울 글 (저장함 등)
        self.note_t = 0.0
        self.rec_since = None        # 녹화 시작 시각 (None = 안 함)

    def rx_fps(self, now):
        self.rx_times = [t for t in self.rx_times if now - t < 2.0]
        return len(self.rx_times) / 2.0


# ------------------------------------------------------------------ 그리기 (ROS 없이 시험 가능)
class Painter:
    def __init__(self):
        from PIL import ImageFont
        self.ko = None
        for p in KO_FONTS:
            if os.path.exists(p):
                try:
                    self.ko = (ImageFont.truetype(p, 18), ImageFont.truetype(p, 26), ImageFont.truetype(p, 14))
                    break
                except Exception:  # noqa: BLE001
                    pass
        if self.ko is None:
            try:
                f = ImageFont.load_default
                self.ko = (f(18), f(26), f(14))
            except TypeError:
                d = ImageFont.load_default()
                self.ko = (d, d, d)
        self.korean = any(os.path.exists(p) for p in KO_FONTS)

    def t(self, ko, en):
        return ko if self.korean else en

    def render(self, st, now=None):
        """-> BGR 이미지 (왼쪽 영상 + 박스, 오른쪽 판)"""
        import cv2
        from PIL import Image, ImageDraw
        now = time.monotonic() if now is None else now
        frame = st.frame if st.frame is not None else np.zeros((480, 640, 3), np.uint8)
        h, w = frame.shape[:2]
        img = frame.copy()

        # 박스 (영상 위)
        for name, c, (x1, y1, x2, y2), guest in st.boxes:
            col = GREEN if guest and c >= st.conf else (ORANGE if guest else GRAY)
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 3 if guest else 1)
            label = f'{name} {c:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(img, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), col, -1)
            cv2.putText(img, label, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        if st.found:
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), GREEN, 8)
        if st.frame is None or now - st.frame_t > 2.0:
            cv2.rectangle(img, (0, h // 2 - 30), (w, h // 2 + 30), (40, 40, 40), -1)

        # 오른쪽 판 (PIL: 한글)
        panel = Image.new('RGB', (W_PANEL, h), (30, 32, 36))
        d = ImageDraw.Draw(panel)
        f, fb, fs = self.ko
        y = 12
        if st.found:
            d.rounded_rectangle((10, y, W_PANEL - 10, y + 46), 8, fill=(40, 160, 70))
            d.text((22, y + 8), self.t('손님 찾음', 'GUEST FOUND'), fill=(255, 255, 255), font=fb)
        else:
            d.rounded_rectangle((10, y, W_PANEL - 10, y + 46), 8, fill=(70, 72, 78))
            d.text((22, y + 8), self.t('찾는 중', 'searching'), fill=(220, 220, 220), font=fb)
        y += 58
        # 연속 막대
        d.text((14, y), self.t(f'연속 {st.hits} / {st.confirm}', f'streak {st.hits} / {st.confirm}'),
               fill=(230, 230, 230), font=f)
        y += 26
        n = max(1, st.confirm)
        bw = (W_PANEL - 28 - (n - 1) * 6) / n
        for i in range(n):
            x = 14 + i * (bw + 6)
            d.rectangle((x, y, x + bw, y + 12), fill=(60, 190, 90) if i < st.hits else (70, 72, 78))
        y += 26

        guests = [b for b in st.boxes if b[3]]
        best = max(guests, key=lambda b: b[1]) if guests else None
        rows = [
            (self.t('기준 conf', 'conf'), f'{st.conf:.2f}   [ / ]'),
            (self.t('손님 클래스', 'class'), st.class_name),
            (self.t('가장 높은 conf', 'best conf'), f'{best[1]:.2f}' if best else '-'),
            (self.t('박스 높이', 'box height'),
             f'{(best[2][3] - best[2][1]) / h * 100:.0f}%' if best else '-'),
            (self.t('가로 위치', 'x offset'),
             f'{((best[2][0] + best[2][2]) / 2 / w - 0.5) * 100:+.0f}%' if best else '-'),
            (self.t('받는 영상', 'rx'), f'{st.rx_fps(now):.1f} fps'),
            (self.t('추론', 'infer'), f'{st.infer_ms:.0f} ms'),
        ]
        for k, v in rows:
            d.text((14, y), k, fill=(160, 165, 175), font=fs)
            d.text((150, y - 2), v, fill=(235, 235, 235), font=f)
            y += 26
        others = [f'{b[0]} {b[1]:.2f}' for b in st.boxes if not b[3]][:3]
        d.text((14, y), self.t('다른 물체: ', 'others: ') + (', '.join(others) or '-'), fill=(160, 165, 175), font=fs)
        y += 30

        # 로봇 안내 상태
        d.line((10, y, W_PANEL - 10, y), fill=(80, 82, 90), width=1)
        y += 10
        d.text((14, y), self.t('로봇 안내 (guide_node)', 'robot guide'), fill=(160, 165, 175), font=fs)
        y += 22
        if st.guide_req:
            a, node, t = st.guide_req
            name = {'greet': self.t('인사 · 손님 기다림', 'greet'), 'watch': self.t('뒤돌아 확인', 'watch'),
                    'arrive': self.t('도착 효과', 'arrive')}.get(a, a)
            d.text((14, y), f'{name} @{node}', fill=(255, 210, 90), font=f)
            d.text((14, y + 24), self.t(f'{now - t:.0f}초 전', f'{now - t:.0f}s ago'), fill=(160, 165, 175), font=fs)
        else:
            d.text((14, y), self.t('요청 없음', 'no request'), fill=(120, 120, 120), font=f)
        y += 46
        if st.guide_res:
            ok, detail, t = st.guide_res
            d.text((14, y), (self.t('결과 OK', 'OK') if ok else self.t('결과 못 찾음', 'NOT SEEN')) + f'  {detail}',
                   fill=(90, 210, 120) if ok else (240, 110, 110), font=f)

        # 아래 안내
        msg = st.note if st.note and now - st.note_t < 2.0 else ''
        if st.paused:
            msg = self.t('멈춤 (p)', 'paused (p)')
        if st.frame is None or now - st.frame_t > 2.0:
            msg = self.t('영상 없음: 로봇 camera_node 확인', 'no video: check camera_node')
        if msg:
            d.text((14, h - 54), msg, fill=(255, 200, 80), font=f)
        d.text((14, h - 26), 's ' + self.t('사진', 'shot') + '  r ' + self.t('녹화', 'rec') + '  p ' +
               self.t('멈춤', 'pause') + '  q ' + self.t('끝', 'quit'), fill=(130, 130, 135), font=fs)

        if st.rec_since is not None:                      # 녹화 중 표시 (영상 왼쪽 위)
            el = int(now - st.rec_since)
            cv2.circle(img, (22, 22), 9, (0, 0, 255), -1)
            cv2.putText(img, f'REC {el // 60:02d}:{el % 60:02d}', (38, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 255), 2, cv2.LINE_AA)

        panel_bgr = np.asarray(panel)[:, :, ::-1]
        return np.hstack([img, panel_bgr])


# ------------------------------------------------------------------ 녹화
class Recorder:
    """두 영상을 같이 씀: view (화면 그대로) + raw (카메라 원본). 새 카메라 프레임이 올 때만 한 장씩"""

    def __init__(self, out_dir, fps=15.0):
        self.out_dir = out_dir
        self.fps = fps
        self.view = self.raw = None
        self.paths = ()
        self.count = 0
        self.last_t = None
        self.sizes = None

    @property
    def on(self):
        return self.view is not None

    def start(self, canvas, raw):
        import cv2
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = time.strftime('%m%d_%H%M%S')
        pv = os.path.join(self.out_dir, f'view_{stamp}.mp4')
        pr = os.path.join(self.out_dir, f'raw_{stamp}.mp4')
        four = cv2.VideoWriter_fourcc(*'mp4v')
        self.sizes = ((canvas.shape[1], canvas.shape[0]), (raw.shape[1], raw.shape[0]))
        self.view = cv2.VideoWriter(pv, four, self.fps, self.sizes[0])
        self.raw = cv2.VideoWriter(pr, four, self.fps, self.sizes[1])
        if not (self.view.isOpened() and self.raw.isOpened()):
            self.view = self.raw = None
            raise RuntimeError('영상 파일을 못 엶 (OpenCV mp4v 코덱)')
        self.paths = (pv, pr)
        self.count = 0
        self.last_t = None
        return self.paths

    def write(self, canvas, raw, frame_t):
        """카메라 프레임이 바뀌었을 때만 씀 (같은 프레임을 화면 갱신마다 반복해 쓰지 않게)"""
        import cv2
        if not self.on or frame_t is None or frame_t == self.last_t:
            return False
        self.last_t = frame_t
        if (canvas.shape[1], canvas.shape[0]) != self.sizes[0]:
            canvas = cv2.resize(canvas, self.sizes[0])
        if (raw.shape[1], raw.shape[0]) != self.sizes[1]:
            raw = cv2.resize(raw, self.sizes[1])
        self.view.write(canvas)
        self.raw.write(raw)
        self.count += 1
        return True

    def stop(self):
        if self.on:
            self.view.release()
            self.raw.release()
        self.view = self.raw = None
        return self.paths, self.count


def extract_frames(video, out_dir, every=5):
    """원본 영상(raw_*.mp4) -> every 장마다 한 장씩 jpg (YOLO 학습 데이터 images/ 폴더용) -> 저장한 수"""
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise SystemExit(f'영상을 못 엶: {video}')
    base = os.path.splitext(os.path.basename(video))[0]
    i = saved = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if i % max(1, every) == 0:
            cv2.imwrite(os.path.join(out_dir, f'{base}_{i:05d}.jpg'), img)
            saved += 1
        i += 1
    cap.release()
    return saved


# ------------------------------------------------------------------ YOLO
class Detector:
    def __init__(self, model_path, st):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        names = self.model.names
        self.names = dict(names.items() if isinstance(names, dict) else enumerate(names))
        if st.class_name not in self.names.values():
            raise SystemExit(f'모델에 클래스 "{st.class_name}" 없음. 있는 것: {list(self.names.values())}')
        self.st = st

    def run(self, frame):
        st = self.st
        t0 = time.monotonic()
        r = self.model(frame, verbose=False, conf=min(0.25, st.conf))[0]   # 기준보다 낮은 것도 회색 · 주황으로 보이게
        ms = (time.monotonic() - t0) * 1000
        boxes = []
        if r.boxes is not None:
            for c, s, xyxy in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
                name = self.names.get(int(c), str(c))
                boxes.append((name, float(s), tuple(int(v) for v in xyxy), name == st.class_name))
        seen = any(g and s >= st.conf for _, s, _, g in boxes)
        with st.lock:
            st.boxes = boxes
            st.boxes_t = time.monotonic()
            st.hits = st.hits + 1 if seen else 0
            st.found = st.hits >= st.confirm
            st.infer_ms = ms


def default_model():
    from .detect_view import default_model as dm
    return dm()


# ------------------------------------------------------------------ 실행
def main():
    import cv2
    argv = sys.argv[1:]
    try:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    except Exception:  # noqa: BLE001
        pass
    ap = argparse.ArgumentParser(description='안내 카메라 확인 창 (PC)')
    ap.add_argument('--topic', default='/camera/image_raw', help='원본 토픽 (압축은 뒤에 /compressed)')
    ap.add_argument('--raw', action='store_true', help='압축 대신 원본 영상')
    ap.add_argument('--model', default='')
    ap.add_argument('--class', dest='cls', default='toy_person')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--confirm', type=int, default=2)
    ap.add_argument('--hz', type=float, default=6.0, help='YOLO 주기 (PC라 로봇 4Hz보다 빨라도 됨)')
    ap.add_argument('--save-dir', default=os.path.expanduser('~/guide_view'))
    ap.add_argument('--record', action='store_true', help='켜자마자 녹화 (r로 끝)')
    ap.add_argument('--rec-fps', type=float, default=15.0, help='녹화 영상 fps (camera_node fps와 같게)')
    ap.add_argument('--extract', default='', help='원본 녹화 영상 -> 사진 (ROS 없이, 이것만 하고 끝)')
    ap.add_argument('--every', type=int, default=5, help='--extract: 몇 장마다 한 장')
    ap.add_argument('--out', default='', help='--extract: 사진 폴더 (기본: <save-dir>/frames)')
    a = ap.parse_args(argv)

    if a.extract:
        out = a.out or os.path.join(a.save_dir, 'frames')
        n = extract_frames(os.path.expanduser(a.extract), out, a.every)
        print(f'{n}장 저장 -> {out}  (손님 없는 장면은 빈 라벨로 넣으면 오인식이 줄어듦)')
        return

    model = a.model or default_model()
    if not model or not os.path.exists(model):
        raise SystemExit(f'모델 없음: {model or "models/best.pt"}  --model로 지정')
    try:
        import ultralytics  # noqa: F401
    except ImportError:
        raise SystemExit('ultralytics 없음: pip install ultralytics')

    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String

    st = ViewState(a.conf, a.confirm, a.cls)
    det = Detector(model, st)
    painter = Painter()

    rclpy.init()
    node = Node('guide_view')
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    def on_img(m):
        if st.paused:
            return
        try:
            if a.raw:
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                img = img.reshape(m.height, m.width, 3)
                img = img[:, :, ::-1].copy() if m.encoding == 'rgb8' else img.copy()
            else:
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        except Exception:  # noqa: BLE001
            return
        if img is None:
            return
        with st.lock:
            st.frame = img
            st.frame_t = time.monotonic()
            st.rx_times.append(st.frame_t)

    def on_req(m):
        try:
            r = json.loads(m.data)
        except ValueError:
            return
        st.guide_req = (r.get('action', '?'), r.get('node', ''), time.monotonic())
        if r.get('action') in ('greet', 'watch'):
            st.guide_res = None

    def on_res(m):
        try:
            r = json.loads(m.data)
        except ValueError:
            return
        st.guide_res = (bool(r.get('ok')), r.get('detail', ''), time.monotonic())

    if a.raw:
        node.create_subscription(Image, a.topic, on_img, qos_profile_sensor_data)
    else:
        node.create_subscription(CompressedImage, a.topic + '/compressed', on_img, qos_profile_sensor_data)
    # /guide/request는 엿듣지 않는다: 구독자가 생기면 fleet_agent가 guide 노드가 있다고 착각함 (guide_run과 같은 이유)
    # 대신 guide 노드가 있을 때만 엿듣는다
    if node.count_subscribers('/guide/request') > 0:
        node.create_subscription(String, '/guide/request', on_req, rel)
    node.create_subscription(String, '/guide/result', on_res, rel)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    def yolo_loop():
        period = 1.0 / max(0.5, a.hz)
        last = 0.0
        while rclpy.ok():
            t0 = time.monotonic()
            with st.lock:
                frame, ft = st.frame, st.frame_t
            if frame is not None and ft != last and not st.paused:
                last = ft
                try:
                    det.run(frame)
                except Exception as e:  # noqa: BLE001
                    print(f'YOLO 실패: {e}')
            time.sleep(max(0.01, period - (time.monotonic() - t0)))

    threading.Thread(target=yolo_loop, daemon=True).start()

    win = 'guide_view'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    rec = Recorder(a.save_dir, a.rec_fps)
    want_rec = a.record

    def toggle_rec(canvas, raw):
        if rec.on:
            (pv, pr), n = rec.stop()
            st.rec_since = None
            st.note, st.note_t = painter.t(f'녹화 끝 {n}장', f'rec saved {n} frames'), time.monotonic()
            print(f'녹화 끝 ({n}장, 약 {n / a.rec_fps:.0f}초)\n  화면: {pv}\n  원본: {pr}')
        else:
            pv, pr = rec.start(canvas, raw)
            st.rec_since = time.monotonic()
            print(f'녹화 시작\n  화면: {pv}\n  원본: {pr}')

    print(f'영상 {a.topic}{"" if a.raw else "/compressed"} 기다리는 중 (ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")})')
    try:
        while True:
            with st.lock:
                canvas = painter.render(st)
                raw, raw_t = st.frame, st.frame_t
            if want_rec and raw is not None:
                want_rec = False
                toggle_rec(canvas, raw)
            if rec.on and raw is not None:
                rec.write(canvas, raw, raw_t)
            cv2.imshow(win, canvas)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord('['):
                st.conf = max(0.05, round(st.conf - 0.05, 2))
            elif k == ord(']'):
                st.conf = min(0.95, round(st.conf + 0.05, 2))
            elif k == ord('p'):
                st.paused = not st.paused
            elif k == ord('r'):
                if raw is None:
                    st.note, st.note_t = painter.t('영상이 없어 녹화 못 함', 'no video'), time.monotonic()
                else:
                    try:
                        toggle_rec(canvas, raw)
                    except RuntimeError as e:
                        st.note, st.note_t = str(e), time.monotonic()
                        print(e)
            elif k == ord('s'):
                os.makedirs(a.save_dir, exist_ok=True)
                p = os.path.join(a.save_dir, time.strftime('view_%H%M%S.jpg'))
                cv2.imwrite(p, canvas)
                st.note, st.note_t = painter.t(f'저장: {p}', f'saved: {p}'), time.monotonic()
                print('저장', p)
    except KeyboardInterrupt:
        pass
    if rec.on:                                   # 녹화 중에 끄면 파일을 닫고 끝 (안 닫으면 영상이 깨짐)
        (pv, pr), n = rec.stop()
        print(f'녹화 저장 ({n}장)\n  화면: {pv}\n  원본: {pr}')
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
