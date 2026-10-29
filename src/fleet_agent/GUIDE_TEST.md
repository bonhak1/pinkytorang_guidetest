# 안내 로봇 시험 (GUIDE_TEST)

안내 기능을 **서버 없이, 기능별로 떼어서** 시험한다. 아래로 갈수록 붙는 것이 많아진다.
앞 단계가 통과해야 다음 단계에서 문제가 생겼을 때 원인을 좁힐 수 있다.

| 단계 | 어디서 | 무엇을 | 도구 |
|---|---|---|---|
| 0 | PC | 안내 판단 로직 전체 (Driver) | `pytest`, `guide_sim` |
| 1 | 로봇 (가만히) | 효과 · 카메라 · YOLO · guide_node | `guide_effect`, `detect_view`, `guide_check` |
| 2 | 로봇 (주행) | 안내 주행 흐름, 손님 결과는 정해 둠 | `guide_run` + `fake_guide` |
| 3 | 로봇 (주행) | 진짜 손님 인식까지 | `guide_run` + `guide_node` |
| 4 | 로봇 + 서버 | 키오스크 → 배정 → 안내 → 복귀 | `main_service` (또는 `main_service_sim`) |

새 파일: `guide_plan.py` (서버와 같은 안내 경로 · Move, ROS 없음), `guide_sim.py`, `guide_run.py`,
`guide_check.py`, `fake_guide.py`, `detect_view.py`, `test/test_guide.py`.
`guide_effects.py`에 `main()` 추가. launch에 `fake_guide` 인자 추가.

```bash
cd ~/ros-repo-2 && colcon build --packages-up-to fleet_agent && source install/setup.bash
```

---

## 0단계 · PC (ROS · 로봇 없이)

```bash
cd ~/ros-repo-2/src/fleet_agent
python3 -m pytest test/ -v                 # test_core.py + test_guide.py
python3 -m fleet_agent.guide_sim           # T1~T6 요약표
python3 -m fleet_agent.guide_sim T3 -v     # 시간순 로그
```

`guide_sim`은 진짜 `core.Driver`를 진짜 `node_graph.yaml` 위에서 돌린다. 서버처럼 Move를 끊어 보내고
(`--lookahead`, 기본 2 = `lock_lookahead`), Nav2 · 회전 · 손님(guide 결과)은 흉내 낸다.

| 시나리오 | 명령 | 기대 |
|---|---|---|
| 정상 | `guide_sim T3` | 인사 D1 → 확인 N1, N3 → 도착 효과 N6 → 복귀 OK |
| 확인에서 놓침 | `guide_sim T5 --lost-at N3 --wait 5 -v` | N3에서 `CMD_FAILED guest_lost:N3:not_seen`, 그 자리에서 복귀 |
| 입구에서 손님 안 옴 | `guide_sim T3 --greet-fail --wait 3` | 안 움직이고 `guest_lost:D1` |
| 서버가 1칸씩 | `guide_sim --lookahead 1` | 확인 노드가 lookahead 2와 같아야 함 |
| 일시정지 | `guide_sim T3 --pause-at N2:3 -v` | 3초 서 있다가 이어서 도착 |
| 사람이 막음 | `guide_sim T3 --obstacle N2:4 -v` | 3초 뒤 LCD "지나갈게요", 비키면 이어 감 (BLOCKED 없음) |
| 물건이 막음 | `guide_sim T3 --obstacle N2:4:STATIC` | 2초 뒤 BLOCKED |
| 확인 기준 | `guide_sim --check-turn 30 --check-gap 0.5` | `guide_check_turn_deg` · `guide_check_gap_m` 바꿔 보기 |

요약표 읽는 법

- **확인노드**: 뒤돌아 손님을 확인한 노드
- **확인방향°**: 뒤돌았을 때 로봇 앞과 손님(직전 노드) 쪽의 차이. 카메라 화각 절반(약 30°) 안이어야 손님이 화면에 든다
- **yaw°**: 도착지에서 테이블 쪽과의 차이

---

## 1단계 · 로봇에서 부품별 (주행 없음)

### 1-1. 효과 (LCD · LED · 부저)

```bash
ros2 run fleet_agent guide_effect welcome      # 또는 arrived / both
```
- [ ] LCD에 웃는 얼굴 + `WELCOME!` / `ARRIVED!`, 들썩 → 윙크 → 활짝
- [ ] 소리 동안 LED가 빨주노초파남보를 2바퀴 돎 (`RAINBOW_LOOPS`)
- [ ] 소리가 끝나면 LCD가 검은 화면으로, LED는 2초 뒤 꺼짐 (`LED_HOLD_SEC`)
- [ ] PC 미리보기: `ros2 run fleet_agent guide_effect --preview /tmp/fx` (얼굴 PNG + 멜로디 WAV)
- [ ] "효과 건너뜀 (라이브러리 없음)"이 나오면 `pinkylib`, `pinky_lcd` 설치 확인

### 1-2. 카메라 + YOLO 인식 튜닝

카메라는 `camera_node` **하나만** 열고 `/camera/image_raw`로 발행한다. guide · docking · detect_view는 구독만 한다.
`robot_system.launch.xml` / `fleet_agent.launch.xml`이 `use_camera:=true`(기본)로 같이 띄운다.
혼자 띄우려면 `ros2 run fleet_agent camera_node --ros-args --params-file <fleet_agent.yaml>`.

- [ ] `ros2 topic hz /camera/image_raw` 가 약 15 Hz
- [ ] 영상이 똑바로 섬 (거꾸로면 `fleet_agent.yaml` camera `rotate_180`)
- [ ] camera_node 로그에 카메라 열기 실패가 없음 (실패하면 다른 프로그램이 카메라를 쥐고 있음: 예전 marker_dock, 다른 카메라 노드)
- [ ] 도킹도 같이 켜서(`use_docking:=true`) 안내 → 도킹 → 안내가 번갈아 돼도 영상이 계속 옴

```bash
ros2 topic hz /camera/image_raw                  # 영상이 오는지 먼저
ros2 run fleet_agent detect_view                 # 계속 돌리며 줄마다 conf · 박스 크기 · 연속 판정
ros2 run fleet_agent detect_view --save /tmp/det --every 5   # 박스 그린 사진 저장
```
손님 인형을 로봇 뒤 **0.3 / 0.5 / 0.8 / 1.2 m**, 정면 · 좌우 30° 에 두고 줄을 본다. Ctrl+C로 요약.

- [ ] 실제 안내 거리(로봇 뒤 약 0.3~0.8 m)에서 `✅ 찾음`이 1초 안에 뜨는가
- [ ] 인형이 없을 때 `찾음`이 안 뜨는가 (toy_car, cone만 있을 때도)
- [ ] 추론 시간 → `infer_hz`를 그보다 낮게 (`fleet_agent.yaml` guide)
- [ ] 결과로 `conf`, `confirm`, `wait_sec` 정하기

PC에서 사진으로: `python3 -m fleet_agent.detect_view --image ./photos`, 웹캠: `--webcam 0`

**PC 화면으로 보기 (`guide_view`)**: 로봇과 같은 `ROS_DOMAIN_ID`인 PC에서
```bash
pip install ultralytics                    # PC에 한 번
ros2 run fleet_agent guide_view            # 압축 영상 받아 PC에서 YOLO, 박스 · conf · 찾음 판정 · 로봇 안내 요청/결과
```
- 초록 박스 = 기준 conf 넘은 손님, 주황 = 손님인데 conf 모자람, 회색 = 다른 물체. 찾음이면 화면 테두리 초록
- `[` `]` conf 바꿔 보기 (정한 값은 `fleet_agent.yaml` guide `conf`에 옮김), `s` 화면 저장, `p` 멈춤, `q` 끝
- 로봇 안내 요청은 guide 노드(guide_node · fake_guide)가 떠 있은 뒤에 guide_view를 켜야 보인다

### 1-3. guide_node 단독

```bash
# 터미널 A
ros2 run fleet_agent guide_node --ros-args \
  --params-file $(ros2 pkg prefix fleet_agent)/share/fleet_agent/config/fleet_agent.yaml \
  -p model_path:=$(ros2 pkg prefix fleet_agent)/share/fleet_agent/models/best.pt
# 터미널 B
ros2 run fleet_agent guide_check all             # 안내 문구 따라 greet -> watch -> arrive -> 취소
ros2 run fleet_agent guide_check watch --repeat 5
```

| 항목 | 기대 |
|---|---|
| greet (인형 있음) | `OK guest_seen` + 환영 효과 |
| watch (인형 없음) | `wait_sec` 뒤 `못 찾음 not_seen` |
| arrive | 도착 효과, 결과 메시지 없음 |
| watch + 2초 뒤 취소 | 결과 **없음** (오면 ❌) |
| 카메라 끄고 watch | `not_seen` 대신 `no_camera` |

---

## 2단계 · 안내 주행 (손님 결과를 정해 둠)

YOLO와 상관없이 **주행 · 인사 대기 · 뒤돌기 · 놓침 처리**만 본다. 서버(main_service)는 끈다.

```bash
# 로봇: 진짜 guide 대신 fake_guide (use_guide는 false)
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky1 start_node:=D1 fake_guide:=true
#   N3 확인에서 손님 놓치게:  ... fake_guide:=true guide_lost_at:=N3
#   요청마다 직접 y/n:       launch는 use_guide:=false로 띄우고 따로  ros2 run fleet_agent fake_guide --ask

# 같은 ROS_DOMAIN_ID의 PC 또는 로봇에서
ros2 run fleet_agent guide_run T3 --plan          # 보낼 Move만 확인 (안 움직임)
ros2 run fleet_agent guide_run T3                 # D1 -> N6 안내 -> 3초 -> D1 복귀
ros2 run fleet_agent guide_run --all              # T1~T6 차례로
```

| 시나리오 | 명령 | 볼 것 |
|---|---|---|
| 기본 | `guide_run T3` | 출발 전 멈춤(인사) → N1 · N3에서 서서 뒤돌기 → N6에서 T3 쪽 보고 섬 → 효과 |
| 놓침 | fake_guide `guide_lost_at:=N3` + `guide_run T5` | N3에서 멈추고 `guest_lost:N3`, 거기서 복귀 |
| lock 경쟁 | `guide_run T3 --hold N1:5` | N1에서 5초 기다렸다가도 꺾이는 곳에서 확인하는지 |
| 일시정지 | `guide_run T3 --pause-at N2:4` | 4초 서 있다가 이어 감 |
| 취소 | `guide_run T5 --cancel-at N3` | N3에 서고 도착 효과 없음 |
| 코너 하나 | `guide_run --to N5 --face T3` | 짧게 한 구간만 |

끝나면 결과표가 나온다.
`확인` 칸의 `N1:1s/12°` = N1에서 1초 확인, 손님 쪽과 12° 차이(TF 기준). 차이가 크면 손님이 카메라 밖이다.

---

## 3단계 · 진짜 손님 인식까지

```bash
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky1 start_node:=D1 use_guide:=true
ros2 run fleet_agent guide_run T3
```
사람이 인형을 들고 로봇을 0.5 m쯤 뒤에서 따라간다.

- [ ] 입구: 인형을 카메라 앞에 보이면 출발
- [ ] 꺾이는 노드: 뒤돌았을 때 인형이 화면 안 (`손님쪽°` 작음), 1~2초 안에 이어 감
- [ ] 인형을 숨기면 `wait_sec` 뒤 `guest_lost`, 그 자리에 섬
- [ ] 도착: 테이블 쪽을 보고 도착 효과 (`yaw°` 작음)

---

## 4단계 · 서버와 함께

시뮬레이터: `main_service_sim <cfg> <배속>`, `SIM_GUEST_LOST=<노드>` 로 안내 실패 흉내.
실제: 서버 + 브릿지 + 로봇, 키오스크(`/kiosk`)에서 테이블 선택 → 매장 UI에서 안내 단계 · 테이블 RESERVED → OCCUPIED · 실패 시 FREE 확인.

---

## 시험하며 찾은 것

`guide_sim`과 `test_guide.py`로 드러난 것 (main `07c5fb6` 기준).

1. **뒤돌아 확인할 때 손님이 아니라 옆을 본다** (약 90°).
   `core._begin_check`가 "지금 yaw + 180°"로 도는데, 꺾이는 노드에 도착할 때 yaw는 서버가 준 *다음 노드 쪽*이다.
   90° 코너에서는 손님이 따라오는 쪽(직전 노드)과 90° 어긋나 손님이 화면에 안 들어오고 `guest_lost`가 날 수 있다.
2. **서버가 1칸씩 보내면 확인을 통째로 건너뛴다.**
   `_reached`에서 다음 노드를 아직 모르면 꺾임 판단을 못 한다. lock 경쟁으로 다음 칸이 늦게 오면 생긴다 (`guide_sim --lookahead 1`, `guide_run --hold N1:5`).
3. **안내 길이 테이블 칸을 지나간다** (설계 확인 필요).
   서버와 같은 최단 경로로 T1~T6 안내 모두 `N2 → T1 → N3`을 지난다 (T1 안내는 자기 테이블 칸을 지나 N3에 섬).
   왼쪽 줄 → 가운데 줄은 T1/T3/T5를 지날 수밖에 없는 그래프라서, 서빙 로봇이 T1에 서 있으면 안내가 기다리거나 돌아간다.

1 · 2는 `guide-fixes.patch`로 고쳤다 (직전 노드 쪽을 보고 돎, 다음 노드를 늦게 받으면 그때 꺾임 판단).
고친 뒤 `test_guide.py`의 "고친 문제" 테스트가 통과한다.
