"""안내 효과: LCD 웃는 얼굴 애니메이션 + LED + 부저 멜로디

play('welcome') : 입구에서 손님을 찾았을 때. 딩동 -> 올라가는 반가운 멜로디, 얼굴이 웃으며 들썩이다 윙크
play('arrived') : 테이블에 도착했을 때. 짧은 팡파레, 활짝 웃는 얼굴 + 반짝이
핑키 라이브러리가 없으면(시험 PC 등) 그냥 건너뛴다. 부저가 sleep으로 2초쯤 걸리므로 guide_node가 별도 스레드에서 부른다.

  LED : 멜로디 길이에 맞춰 빨주노초파남보를 RAINBOW_LOOPS(2)번 돌고, 소리가 끝나면 마지막 색 그대로
        LED_HOLD_SEC(2초) 뒤 꺼짐 (play는 소리가 끝나면 바로 돌아오고 LED 끄기는 뒤에서. 새 효과가 오면 이전 것은 그만)
  LCD : 멜로디 동안 웃는 얼굴 애니메이션, 끝나면 검은 화면 (백라이트는 그대로 -> 서버 LCD 문구가 다시 보임)

혼자 시험 (ROS 필요 없음)
  ros2 run fleet_agent guide_effect welcome|arrived|both        # 로봇에서 진짜로
  ros2 run fleet_agent guide_effect --preview /tmp/fx           # PC: 얼굴 PNG + 멜로디 WAV 저장 (로봇 없이 보고 듣기)
"""
import os
import threading
import time

W, H = 320, 240

# (주파수 Hz 또는 None = 쉼, 초). 멜로디는 새로 만든 것 (기존 곡 아님)
#   welcome: 딩-동 (문 열리는 소리) -> 쉼 -> 도-미-솔 올라가며 -> 높은 도에서 한 번 튕기고 길게
#   arrived: 짧게 올라가는 팡파레 -> 끝음 두 번 (짠-짜잔)
MELODY = {
    'welcome': [(1319, .16), (1047, .26), (None, .10),
                (784, .09), (988, .09), (1175, .09), (1568, .16),
                (None, .05), (1319, .09), (1568, .42)],
    'arrived': [(1047, .09), (1319, .09), (1568, .09), (2093, .20),
                (None, .07), (1568, .10), (2093, .38)],
}
# LED 무지개 (빨주노초파남보). 멜로디 전체 길이를 나눠 RAINBOW_LOOPS번 돈다 (음 박자와 상관없이 고르게)
RAINBOW = [(255, 0, 0), (255, 127, 0), (255, 255, 0), (0, 255, 0), (0, 0, 255), (75, 0, 130), (148, 0, 211)]
RAINBOW_LOOPS = 2        # 소리 동안 무지개를 몇 번 돌지
LED_HOLD_SEC = 2.0       # 소리가 끝나고 LED를 이만큼 더 켜 둔 뒤 끔
NOTE_GAP = 0.03          # 음 사이 살짝 끊음
LCD_BLACK = True         # 끝나면 LCD를 검은 화면으로 (백라이트는 끄지 않음)

BG = {'welcome': (255, 232, 120), 'arrived': (190, 236, 255)}
FACE = (255, 205, 45)
LINE = (55, 40, 10)
CHEEK = (255, 140, 140)
MOUTH = (150, 30, 40)
TONGUE = (255, 110, 120)
SPARK = (255, 140, 0)        # 반짝이 바깥 (배경과 잘 구분되게 주황)
SPARK_IN = (255, 255, 255)


# ------------------------------------------------------------------ 그림
def _font(size):
    from PIL import ImageFont
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
              '/usr/share/fonts/TTF/DejaVuSans-Bold.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    try:
        return ImageFont.load_default(size)          # Pillow >= 10.1
    except TypeError:
        return ImageFont.load_default()


def _star(d, cx, cy, r, color):
    """반짝이 (네 갈래 별)"""
    k = r * 0.28
    d.polygon([(cx, cy - r), (cx + k, cy - k), (cx + r, cy), (cx + k, cy + k),
               (cx, cy + r), (cx - k, cy + k), (cx - r, cy), (cx - k, cy - k)], fill=color)


def face(kind='welcome', frame=0):
    """웃는 얼굴 한 장 (320x240). frame으로 동작을 바꾼다
    0 = 활짝 웃음(눈 ^^)  1 = 위로 들썩  2 = 윙크 + 반짝이  3 = 입 크게 + 반짝이(마지막)"""
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (W, H), BG.get(kind, BG['welcome']))
    d = ImageDraw.Draw(img)
    dy = -8 if frame == 1 else 0                  # 들썩
    cx, cy, r = W // 2, 140 + dy, 86

    # 얼굴
    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=FACE, outline=LINE, width=5)
    # 볼
    for sx in (-1, 1):
        x = cx + sx * 58
        d.ellipse((x - 18, cy + 6, x + 18, cy + 26), fill=CHEEK)

    # 눈: 웃는 눈(^ 모양 호). 윙크면 오른쪽 눈은 >< 대신 찡긋 선
    ey = cy - 26
    for sx in (-1, 1):
        x = cx + sx * 36
        if frame == 2 and sx == 1:
            d.line((x + 16, ey - 4, x - 14, ey + 5), fill=LINE, width=7)          # 찡긋 (< 모양, 꼭짓점이 코 쪽)
            d.line((x + 16, ey + 14, x - 14, ey + 5), fill=LINE, width=7)
        else:
            d.arc((x - 18, ey - 12, x + 18, ey + 20), 200, 340, fill=LINE, width=8)  # ^

    # 입: 활짝 웃는 입(반달) + 혀
    mw = 52 if frame == 3 else 44
    top, bot = cy + 18, cy + (70 if frame == 3 else 62)
    d.chord((cx - mw, top - (bot - top), cx + mw, bot), 0, 180, fill=MOUTH, outline=LINE, width=4)
    d.chord((cx - mw * 0.55, bot - 22, cx + mw * 0.55, bot + 6), 180, 360, fill=TONGUE)
    d.arc((cx - mw, top - (bot - top), cx + mw, bot), 0, 180, fill=LINE, width=4)

    # 반짝이
    if frame in (2, 3):
        for x, y, s in ((cx + r + 14, cy - r + 10, 16), (cx - r - 12, cy - r + 30, 11), (cx + r - 4, cy + 40, 9)):
            _star(d, x, y, s, SPARK)
            _star(d, x, y, s * 0.45, SPARK_IN)

    # 글자
    text = 'WELCOME!' if kind == 'welcome' else 'ARRIVED!'
    f = _font(30)
    try:
        tw = d.textlength(text, font=f)
    except AttributeError:
        tw = len(text) * 16
    d.text(((W - tw) / 2, 4), text, fill=LINE, font=f)
    return img


def frames_for(kind):
    """음마다 보여 줄 frame 번호 (melody와 같은 길이). None = 그대로"""
    n = len(MELODY[kind])
    if kind == 'welcome':
        seq = [0, None, None, 1, 0, 1, 0, None, 2, 3]       # 딩동엔 가만히 -> 올라갈 때 들썩 -> 윙크 -> 활짝
    else:
        seq = [0, 1, 0, 1, None, 2, 3]
    return (seq + [None] * n)[:n]


# ------------------------------------------------------------------ 진짜 로봇
_led_lock = threading.Lock()
_led_stop = threading.Event()      # 지금 돌고 있는 무지개 LED를 멈추는 신호
_led_thread = None


def _stop_rainbow():
    """돌고 있는 무지개 LED를 멈춤 (새 효과가 시작될 때)"""
    global _led_thread
    with _led_lock:
        _led_stop.set()
        t, _led_thread = _led_thread, None
    if t and t is not threading.current_thread():
        t.join(timeout=1.0)


def _led_off(led, log):
    """LED 끄기: 검정 두 번 (한 번 놓쳐도 꺼지게) + 라이브러리에 clear가 있으면 그것도"""
    for _ in range(2):
        led.fill((0, 0, 0))
        led.show()
        time.sleep(0.05)
    for name in ('clear', 'off'):
        f = getattr(led, name, None)
        if callable(f):
            try:
                f()
            except Exception as e:  # noqa: BLE001
                log(f'LED {name}() 실패: {e}')
            break


def _hold_then_off(led, buzzer, hold, stop, log):
    """소리 뒤: 마지막 색 그대로 hold 초 -> LED 끔 -> 그다음에야 부저 정리(clean)
    (clean이 GPIO를 정리하면 그 뒤 LED · LCD 명령이 안 먹으므로 반드시 맨 마지막)"""
    try:
        if not stop.wait(hold):
            _led_off(led, log)
            log('LED 꺼짐')
    except Exception as e:  # noqa: BLE001
        log(f'LED 끄기 실패: {e}')
    finally:
        if not stop.is_set():          # 새 효과가 이미 시작했으면 GPIO를 건드리지 않음
            try:
                buzzer.clean()
            except Exception as e:  # noqa: BLE001
                log(f'부저 정리 실패: {e}')


def _lcd_black(lcd):
    """LCD 검은 화면 (잔상 없게 두 번)"""
    from PIL import Image
    black = Image.new('RGB', (W, H), (0, 0, 0))
    for _ in range(2):
        lcd.img_show(black)
        time.sleep(0.05)


def rainbow_at(elapsed, total, loops=RAINBOW_LOOPS):
    """소리 시작부터 elapsed초일 때 LED 색 번호 (0 ~ 7*loops-1)"""
    n = len(RAINBOW) * loops
    return min(n - 1, int(elapsed / max(total, 1e-3) * n))


def play(kind, log=print, hold=None):
    """효과 한 번. 소리가 끝나면 돌아온다 (LED는 뒤에서 hold초 뒤 꺼짐). -> LED 끄는 스레드 (없으면 None)"""
    global _led_thread
    try:
        from pinkylib import LED, Buzzer
        from pinky_lcd import LCD
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 건너뜀 (라이브러리 없음: {e})')
        return None
    hold = LED_HOLD_SEC if hold is None else hold
    _stop_rainbow()                                           # 이전 효과의 무지개 LED 그만
    try:
        notes = MELODY.get(kind, MELODY['welcome'])
        imgs = {k: face(kind, k) for k in set(f for f in frames_for(kind) if f is not None)}   # 미리 그려 둠
        lcd = LCD()
        lcd.set_backlight(255)
        lcd.img_show(imgs.get(0) or face(kind, 0))
        led = LED()
        b = Buzzer()
        b.buzzer_start()
        total = sum(sec + NOTE_GAP for _, sec in notes)       # 소리 전체 길이 -> 무지개 2바퀴를 여기에 고르게
        start = time.monotonic()
        shown = [-1]

        def led_until(t_end):
            """t_end까지 기다리며 LED를 무지개 차례대로 (색이 바뀔 때만 보냄)"""
            while True:
                now = time.monotonic()
                k = rainbow_at(now - start, total)
                if k != shown[0]:
                    led.fill(RAINBOW[k % len(RAINBOW)])
                    led.show()
                    shown[0] = k
                if now >= t_end:
                    return
                time.sleep(min(0.02, t_end - now))

        t_note = start
        for (hz, sec), fr in zip(notes, frames_for(kind)):
            if hz:
                b.set_buzzer_freq(hz)
                b.set_buzzer_duty(35)
            else:
                b.set_buzzer_duty(0)
            if fr is not None and fr in imgs:
                lcd.img_show(imgs[fr])                       # LCD 그리는 시간도 음 길이 안에서
            t_note += sec
            led_until(t_note)
            b.set_buzzer_duty(0)
            t_note += NOTE_GAP                                # 음 사이 살짝 끊어서 또렷하게
            led_until(t_note)
        b.set_buzzer_duty(0)
        b.buzzer_stop()                                       # 소리만 멈춤. clean(GPIO 정리)은 LED 끈 뒤 맨 마지막
        if LCD_BLACK:
            _lcd_black(lcd)                                   # 소리 끝 = 얼굴 지우고 검은 화면 (정리 전에)
        led.fill(RAINBOW[-1])                                 # 2바퀴 끝 = 보라 (마지막 색 그대로 둠)
        led.show()
        stop = threading.Event()
        t = threading.Thread(target=_hold_then_off, args=(led, b, hold, stop, log), daemon=True)
        with _led_lock:
            globals()['_led_stop'] = stop
            _led_thread = t
        t.start()
        return t
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 실패: {e}')
        return None


# ------------------------------------------------------------------ PC 미리보기
def preview(out_dir, kinds=('welcome', 'arrived')):
    """얼굴 PNG (frame별 + 한 장에 모은 것) + 멜로디 WAV. 로봇 · 부저 없이 확인용"""
    import math
    import struct
    import wave

    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    out = []
    for kind in kinds:
        frs = [0, 1, 2, 3]
        sheet = Image.new('RGB', (W * len(frs) + 10 * (len(frs) - 1), H), (255, 255, 255))
        for i, fr in enumerate(frs):
            im = face(kind, fr)
            im.save(os.path.join(out_dir, f'{kind}_{fr}.png'))
            sheet.paste(im, (i * (W + 10), 0))
        p = os.path.join(out_dir, f'{kind}_faces.png')
        sheet.save(p)
        out.append(p)

        rate = 22050
        data = bytearray()
        for hz, sec in MELODY[kind]:
            n = int(rate * sec)
            for k in range(n):
                if hz:
                    env = min(1.0, k / 200.0, (n - k) / 400.0)            # 딸깍 소리 줄이기
                    v = (1 if math.sin(2 * math.pi * hz * k / rate) >= 0 else -1) * 0.25 * env   # 부저 = 네모파
                else:
                    v = 0.0
                data += struct.pack('<h', int(v * 32767))
            data += struct.pack('<h', 0) * int(rate * .03)
        p = os.path.join(out_dir, f'{kind}.wav')
        with wave.open(p, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(bytes(data))
        out.append(p)
    return out


def main():
    """ros2 run fleet_agent guide_effect welcome|arrived|both  /  --preview <폴더>"""
    import sys
    args = sys.argv[1:]
    if args[:1] == ['--preview']:
        out = args[1] if len(args) > 1 else 'guide_fx_preview'
        for p in preview(out):
            print(p)
        return
    kinds = args or ['both']
    kinds_all = [x for k in kinds for x in (['welcome', 'arrived'] if k == 'both' else [k])]
    for k in kinds:
        for kind in (['welcome', 'arrived'] if k == 'both' else [k]):
            if kind not in MELODY:
                sys.exit(f'모르는 효과: {kind} (welcome / arrived / both, 또는 --preview <폴더>)')
            total = sum(s + .03 for _, s in MELODY[kind])
            print(f'▶ {kind}: 웃는 얼굴 애니메이션 · 무지개 LED {RAINBOW_LOOPS}바퀴 · 멜로디 {len(MELODY[kind])}음 ({total:.1f}초)')
            t0 = time.monotonic()
            led_t = play(kind)
            print(f'  소리 끝 ({time.monotonic() - t0:.1f}초): LCD 검은 화면, LED는 {LED_HOLD_SEC:.0f}초 뒤 꺼짐')
            if led_t and kind == kinds_all[-1]:
                try:
                    led_t.join()                                 # 마지막 효과는 LED가 꺼질 때까지 기다림
                except KeyboardInterrupt:
                    _stop_rainbow()
                    print('  (Ctrl+C) LED 그대로 멈춤')
            else:
                time.sleep(1.5)


if __name__ == '__main__':
    main()
