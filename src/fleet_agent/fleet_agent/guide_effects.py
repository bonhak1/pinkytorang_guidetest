"""안내 효과: LCD 웃는 얼굴 + LED + 짧은 부저 멜로디 (안내 담당자 코드 정리본)

play('welcome') / play('arrived'). 핑키 라이브러리가 없으면(시험 PC 등) 그냥 건너뛴다.
부저가 sleep으로 1초쯤 걸리므로 guide_node가 별도 스레드에서 부른다.
"""
import time

WELCOME = [(523, .16), (659, .16), (784, .16), (1047, .30)]
ARRIVED = [(784, .15), (1047, .18), (784, .12), (1047, .32)]


def play(kind, log=print):
    try:
        from pinkylib import LED, Buzzer
        from pinky_lcd import LCD
        from PIL import Image, ImageDraw
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 건너뜀 (라이브러리 없음: {e})')
        return
    try:
        lcd = LCD()
        image = Image.new('RGB', (320, 240), (255, 224, 80))
        d = ImageDraw.Draw(image)
        d.ellipse((55, 25, 265, 235), fill=(255, 201, 38), outline=(60, 50, 0), width=5)
        d.ellipse((105, 85, 125, 110), fill=(25, 25, 25))
        d.ellipse((195, 85, 215, 110), fill=(25, 25, 25))
        d.arc((105, 105, 215, 190), 10, 170, fill=(25, 25, 25), width=8)
        d.text((85, 8), 'WELCOME!' if kind == 'welcome' else 'ARRIVED!', fill=(25, 25, 25))
        lcd.set_backlight(255)
        lcd.img_show(image)

        led = LED()
        led.fill((255, 190, 0))
        led.show()
        b = Buzzer()
        b.buzzer_start()
        b.set_buzzer_duty(35)
        for hz, sec in (WELCOME if kind == 'welcome' else ARRIVED):
            b.set_buzzer_freq(hz)
            time.sleep(sec)
            b.set_buzzer_duty(0)
            time.sleep(.05)
            b.set_buzzer_duty(35)
        b.set_buzzer_duty(0)
        b.buzzer_stop()
        b.clean()
        led.fill((0, 0, 0))
        led.show()
    except Exception as e:  # noqa: BLE001
        log(f'안내 효과 실패: {e}')


def main():
    """효과만 따로 시험 (ROS 필요 없음):  ros2 run fleet_agent guide_effect welcome|arrived|both"""
    import sys
    kinds = sys.argv[1:] or ['both']
    for k in kinds:
        for kind in (['welcome', 'arrived'] if k == 'both' else [k]):
            if kind not in ('welcome', 'arrived'):
                sys.exit(f'모르는 효과: {kind} (welcome / arrived / both)')
            print(f'▶ {kind}: LCD {"WELCOME!" if kind == "welcome" else "ARRIVED!"} 얼굴 · LED 노랑 · 부저 멜로디')
            t0 = time.monotonic()
            play(kind)
            print(f'  끝 ({time.monotonic() - t0:.1f}초). LCD가 그대로면 정상 (효과 뒤 LCD는 다음 문구까지 유지)')
            time.sleep(0.5)


if __name__ == '__main__':
    main()
