"""안내 효과: LCD 웃는 얼굴 애니메이션 + LED + 부저 멜로디

play('welcome') : 입구에서 손님을 찾았을 때. 딩동 -> 올라가는 반가운 멜로디, 얼굴이 웃으며 들썩이다 윙크
play('arrived') : 테이블에 도착했을 때. 짧은 팡파레, 활짝 웃는 얼굴 + 반짝이
핑키 라이브러리가 없으면(시험 PC 등) 그냥 건너뛴다. 부저가 sleep으로 2초쯤 걸리므로 guide_node가 별도 스레드에서 부른다.

혼자 시험 (ROS 필요 없음)
  ros2 run fleet_agent guide_effect welcome|arrived|both        # 로봇에서 진짜로
  ros2 run fleet_agent guide_effect --preview /tmp/fx           # PC: 얼굴 PNG + 멜로디 WAV 저장 (로봇 없이 보고 듣기)
"""
import os
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
# 음마다 LED 색 (따뜻한 색을 돌아가며)
LED_COLORS = [(255, 190, 0), (255, 120, 40), (255, 90, 140), (255, 200, 60), (255, 150, 0)]

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
def play(kind, log=print):
    try:
        from pinkylib import LED, Buzzer
        from pinky_lcd import LCD
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 건너뜀 (라이브러리 없음: {e})')
        return
    try:
        notes = MELODY.get(kind, MELODY['welcome'])
        imgs = {k: face(kind, k) for k in set(f for f in frames_for(kind) if f is not None)}   # 미리 그려 둠
        lcd = LCD()
        lcd.set_backlight(255)
        lcd.img_show(imgs.get(0) or face(kind, 0))
        led = LED()
        b = Buzzer()
        b.buzzer_start()
        for i, ((hz, sec), fr) in enumerate(zip(notes, frames_for(kind))):
            t0 = time.monotonic()
            if hz:
                b.set_buzzer_freq(hz)
                b.set_buzzer_duty(35)
                led.fill(LED_COLORS[i % len(LED_COLORS)])
                led.show()
            else:
                b.set_buzzer_duty(0)
            if fr is not None and fr in imgs:
                lcd.img_show(imgs[fr])                       # LCD 그리는 시간도 음 길이 안에서
            time.sleep(max(0.0, sec - (time.monotonic() - t0)))
            b.set_buzzer_duty(0)
            time.sleep(.03)                                   # 음 사이 살짝 끊어서 또렷하게
        b.buzzer_stop()
        b.clean()
        time.sleep(0.4)
        led.fill((0, 0, 0))
        led.show()                                            # 얼굴은 다음 문구까지 그대로 둔다
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 실패: {e}')


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
    for k in kinds:
        for kind in (['welcome', 'arrived'] if k == 'both' else [k]):
            if kind not in MELODY:
                sys.exit(f'모르는 효과: {kind} (welcome / arrived / both, 또는 --preview <폴더>)')
            total = sum(s + .03 for _, s in MELODY[kind])
            print(f'▶ {kind}: 웃는 얼굴 애니메이션 · LED · 멜로디 {len(MELODY[kind])}음 ({total:.1f}초)')
            t0 = time.monotonic()
            play(kind)
            print(f'  끝 ({time.monotonic() - t0:.1f}초). 얼굴은 LCD에 그대로 남는 게 정상')
            time.sleep(0.8)


if __name__ == '__main__':
    main()
