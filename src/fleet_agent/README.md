# fleet_agent — 로봇 쪽 에이전트 (Python, rclpy)

메인 서비스가 보낸 Move를 받아 노드 단위로 주행하고, 결과를 `fleet/state` · `fleet/event`로 돌려준다.
규칙은 `fleet_msgs/README.md` 5장 (로봇 규칙)을 따른다.

```
            fleet_msgs (domain_bridge)                       로봇 안
 Main  ── /fleet/move · control · display ──▶ ┌──────────────┐ ──▶ Nav2 (navigate_to_pose)   FORWARD
       ◀── /fleet/state · event ───────────── │  fleet_agent │ ──▶ /cmd_vel                  REVERSE · 회전
                                              │  (Driver)    │ ──▶ /lcd/text                 LCD
                                              └──────────────┘
                     /docking/request ▲ ▼ /docking/result        /obstacle/direction ▼ ▲ /obstacle/status
                                 ┌──────────┐                        ┌──────────┐
                                 │ docking  │ ◀── CAM                │ obstacle │ ◀── /scan (LiDAR)
                                 └──────────┘                        └──────────┘
```

| 파일 | 노드 | 하는 일 |
|---|---|---|
| `fleet_agent/core.py` | (ROS 없음) | `Driver` 주행 상태기계, `StepQueue`, `ReverseControl`, `ObstacleClass`, `BlockPolicy` |
| `fleet_agent/agent_node.py` | `fleet_agent` | fleet 토픽 · Nav2 액션 · TF 위치 · LCD · 배터리를 `Driver`에 이어 줌 |
| `fleet_agent/obstacle_node.py` | `obstacle` | 진행 방향 LiDAR 부채꼴로 막힘 판단, 사람(DYNAMIC) / 물건(STATIC) 구분 |
| `fleet_agent/docking_node.py` | `docking` | 정밀 정차 요청을 받아 방법별로 맡김 (스레드에서 실행, 시간 초과 · 취소 처리) |
| `fleet_agent/docking/marker_dock.py` | (모듈 + `marker_dock` 실행) | 마커 도킹 v4.3 (`Docker`): dock(D1~D4) · align(N14 마커 5 → 6). 혼자 실행으로 보정 · 목표 저장 |
| `fleet_agent/docking/line_dock.py` | (모듈) | tape: P1 E자 테이프로 바구니를 픽업대에 · tape_exit: 새 선까지 빼기 (바닥 IR 3개) |
| `fleet_agent/docking/undock.py` | (모듈) | undock: D3/D4에서 마커로 N16/N15까지 + 회전 + 방위각 정렬 (주방 통로 후진 축) |
| `fleet_agent/docking/marker_align.py` | (`marker_align` 실행) | 정렬 기준 저장 (`--record`) · 시험 |

## 노드 사이 인터페이스 (로봇 안, std_msgs/String JSON)

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/docking/request` | agent → docking | `{"id", "node", "method", ...precision_stops.yaml 값}` |
| `/docking/result` | docking → agent | `{"id", "node", "ok", "detail"}` (`timeout_sec` 안에 못 하면 ok + "timeout") |
| `/docking/cancel` | agent → docking | `{"id"}` (정지 · 취소 때) |
| `/obstacle/direction` | agent → obstacle | `FORWARD` / `REVERSE` / `STOP` (어느 쪽을 볼지) |
| `/obstacle/status` | obstacle → agent | `{"blocked", "kind", "distance", "direction"}` |
| `/lcd/text` | agent → LCD 드라이버 | 띄울 문구 (카운트다운 포함) |

## 주행 규칙 (Driver)

- Move의 waypoint를 줄(StepQueue)에 쌓는다. `REPLACE`면 비우고 새로, `APPEND`면 같은 cmd_id일 때만 뒤에
- `FORWARD`는 Nav2로 간다. 중간 노드는 `pass_tolerance` 안에 들어오면 바로 다음 목표로 넘어간다 (위치만 통과)
- `REVERSE`는 `/cmd_vel` 직선 후진 (방향 P제어). 카메라가 못 보므로 장애물은 LiDAR만
- 앞 Move와 drive가 다르면 그 노드에서 멈추고 진행 방향으로 제자리 회전한 뒤 출발 (후진이면 뒤가 진행 방향)
- 노드마다 `NODE_REACHED`. `align` 노드(N14)는 지날 때 docking에 맞춤을 맡기고 이어 간다
- 줄이 비면 `done_grace_sec`(0.5초) 기다린 뒤 끝 처리: 후진으로 왔으면 노드 yaw로 회전 → `dock` / `tape`면 docking → `CMD_DONE`
- 장애물 (obstacle `blocked`): 멈추고 `state = BLOCKED`
  - 사람: 3초 기다림 → 5초 LCD "지나갈게요" → 서버에 `BLOCKED` (`reason = "DYNAMIC:N14->N15"`)
  - 물건: 2초 확인 → `BLOCKED` (`reason = "STATIC:..."`)
  - 길이 비면 그 노드부터 다시 출발
- `PAUSE`는 멈춤(줄 유지), `RESUME`은 이어서, `CANCEL`은 줄 비우고 정지
- Nav2 실패면 `CMD_FAILED` + `state = ERROR`. 서버가 RESUME을 보내면 남은 경로를 다시 받는다

## 정밀 정차 (docking)

| method | 노드 | 구현 |
|---|---|---|
| `dock` | D1~D4 | `marker_dock.Docker.dock()` (마커 `marker`의 목표 파일로) → `back` m 후진 |
| `align` | N14 | 같은 `Docker`로 `markers`를 차례로 (마커가 안 보이면 30°씩 돌며 찾기) |
| `tape` | P1 | `line_dock.LineDocker.run()` — E자 테이프: 가로선 → X → 시계 90° → 동 선까지 후진 (T1~T6은 아직 미구현) |
| leave `undock` | D3 · D4를 떠날 때 | `undock.undock()` — 마커로 N16/N15까지, 왼쪽 90°, 방위각 정렬. 그 뒤 후진은 이 축 그대로 (`hold_axis`) |
| leave `tape_exit` | P1을 떠날 때 | `line_dock.LineDocker.leave()` — 앞으로 가며 선을 세서 새 선에서 정지 → Nav2 |

카메라: `camera_node` 하나만 카메라를 열고 `/camera/image_raw`(bgr8, 180도 돌린 것)를 발행한다.
docking · marker_dock · marker_align · guide · detect_view는 모두 구독만 한다 (fleet_agent.launch.xml `use_camera:=true` 기본).

마커 준비 (로봇마다. camera_node가 떠 있어야 함 = `ros2 run fleet_agent camera_node` 또는 launch.
camera_node 없이 직접 열려면 `--direct`, 이때는 camera_node · docking 노드를 꺼 둔다):
```bash
ros2 run fleet_agent marker_dock --calibrate          # 처음 한 번: 카메라 거리 보정 -> ~/dock_cam.json
ros2 run fleet_agent marker_dock --record --id 3      # 도킹 자리마다: 로봇을 최종 위치에 두고 -> ~/dock_target_id3.json
ros2 run fleet_agent marker_dock --dry-run --id 3     # 계획만 보기
```
- 목표 파일은 마커 ID별이라, D3 자리에서 `--id 3`, N14에서 `--id 5`(동쪽 보고) · `--id 6`(남쪽 보고)처럼 저장한다
- 도킹 로그 · 사진은 `~/dock_logs/`에 도킹마다 남는다

## 주방 통로 · 픽업 (D3/D4 → N15/N16 → P2 → P1 → N4)

```
D4 ──undock──▶ N15 ──(축 고정 후진)──▶ P2(대기) ──▶ P1 ──tape──▶ 바구니 삽입 ──[적재 완료]──tape_exit──▶ 새 선 ──Nav2──▶ N4
     마커2 + 왼쪽 90° + 마커0 정렬          odom 축 유지         가로선 · X · 시계 90° · 동 선        선 세기
```
- 서버 명령은 그대로다: `FORWARD [N15]` → `REVERSE [P2]` → `REVERSE [P1]` → (적재) → `FORWARD [N4 ...]`
- `D4.leave`(undock)가 N15까지 대신 가므로 그 Move에서 Nav2는 안 쓴다. 이어지는 후진은 마커로 맞춘 축 그대로 가고,
  N15에서 돌지 않으며 P2 · P1 끝 yaw 회전도 하지 않는다 (가로선 · 회전은 tape가 함)
- P2 -> P1은 좌표까지 가지 않고 **tape에 바로 맡긴다** (tape_approach): P2에서 가로선(검검검)까지 8 cm/s로 후진해 멈춘다.
  그래서 P1 좌표가 몇 cm 틀려도 상관없다 (최대 1.2 m 안에 가로선이 없으면 실패로 정지)
- 축 고정 후진의 방향 유지는 **odom** 기준 (AMCL이 위치를 고칠 때 로봇이 따라 꺾이지 않게). 후진 속도 `reverse_speed` 0.08
- 바닥 센서: `pinky_sensor_adc main_node` (robot_system이 `use_ir:=true`로 켬). 바닥 230~250, 테이프 4050~4095, 2000 넘으면 검정

E자 테이프 (지도 방향, 북쪽 위 · 픽업대 오른쪽)
```
  새 선   서 선  동 선
   ┃      ┃      ┃       세로선 3개 (사진상 간격 새~서 약 14 cm, 서~동 약 7 cm)
   ┗━━━━━━┻━━━━━━┛       가로선 = 검검검 (메인선은 1~2개만 검정이라 착각 안 함)
          ┃              메인선 (복도, 있어도 됨)
```

준비 (로봇마다 한 번, fleet_agent는 끄고)
```bash
ros2 run fleet_agent marker_dock --calibrate                           # 카메라 보정 (처음 한 번)
ros2 run fleet_agent marker_dock --id 2 --name n15 --record           # 로봇을 N15에, 서쪽 D4 마커 보게
ros2 run fleet_agent marker_dock --id 2 --record                      # (D4 도킹 목표) 로봇을 D4 최종 자리에
# D4에서 언도킹 + 왼쪽 90도 돈 자세(반듯한지 확인)에서:
ros2 run fleet_agent marker_align --id 0 --size 0.05 --name n14 --record
```
켜기
```bash
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky2 start_node:=D4 \
  use_docking:=true precision_stops:=$(ros2 pkg prefix fleet_msgs)/share/fleet_msgs/docs/precision_stops.yaml
```
값 바꾸기: `precision_stops.yaml`의 `P1.x_after`(가로선 뒤 거리), `P1.leave.exit_legs`, `D4.leave` 마커 · 이름.
그 밖의 기본값은 `docking/line_dock.py`의 `DEFAULTS` (속도 · 거리), `docking/undock.py` 맨 위.
D3는 `D3.leave`에 같은 형식으로 (`target: n16, to: N16`, 정렬 기준 따로 저장) 넣으면 된다.

## 한 번에 켜기 (robot_system.launch.xml)

bringup -> (2초) Nav2(맵 pinkytorang_map_v4, 설정 config4) -> 초기 위치 + fleet_agent 를 한 명령으로 켠다.
```bash
export ROS_DOMAIN_ID=52
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky2 start_node:=D2
```
- 초기 위치: `initial_pose`가 node_graph.yaml의 start_node 좌표 · yaw로 /initialpose를 보낸다 (RViz 2D Pose Estimate 대신).
  D 좌표가 안 맞으면 `set_initial_pose:=false`로 끄고 RViz로 맞춘다
- 맵 · Nav2 설정 바꾸기: `map:=... params_file:=...` (기본 ~/maps/pinkytorang_map_v4.yaml, ~/params/config4.yaml)
- Nav2에서 tf 에러가 계속 나면 `nav_delay:=5`
- 도킹 없이 N 노드에서 시험: `start_yaw:=<rad>`(로봇 앞 방향), `spin_deg:=360`(초기 위치 뒤 제자리 한 바퀴로 AMCL 맞춤)
  서버는 `main_service_test.yaml`로 켠다 (켤 때 노드에서 대기, 작업 끝나면 거기로 돌아감)

## 안내 (guide_node) - 손님 확인

서버는 안내 Move에 `guide: true`를 켜고, 모든 Move에 `goal_node`(최종 도착지)를 채운다 (fleet_msgs v9). 나머지는 fleet_agent가 정한다

| 언제 | 로봇 동작 |
|---|---|
| 안내 Move를 처음 받았을 때 (출발 전) | 앞 카메라로 손님 기다림 -> 찾으면 환영 효과 -> 출발 |
| 꺾이는 노드 (방향이 `guide_check_turn_deg` 45도 넘게 바뀜) | 그 노드에서 섬 -> 뒤로 돎 -> 손님 확인 -> 다음 노드 쪽으로 돎 -> 이어 감 |
| `goal_node`에 도착 (안내 자리, 예: T3 안내면 N6) | 테이블 쪽으로 돎 -> 도착 효과 (LCD 얼굴 · LED · 부저) |

- 정밀 정차(도킹 마커 · 테이프)와 나가기는 `goal_node`에 도착했을 때만 한다 (`goal_node`가 비면 예전처럼 waypoints 끝). 안내 중엔 안 한다
- 바로 전 인사 · 확인 자리에서 `guide_check_gap_m`(0.8m) 안의 꺾임은 건너뛴다 (맵이 작아서). 지금 맵 기준 확인 자리:
  T1·T2 = N1 / T3 = N1 / T4 = N1, N3 / T5 = N1, N8 / T6 = N1, N3, N9
- 못 찾으면(`wait_sec` 30초) 그 자리에 멈추고 `CMD_FAILED reason="guest_lost:<노드>:<이유>"` -> 서버가 안내 실패 처리, 로봇은 집으로
- YOLO는 guide 노드(별도 프로세스)에서 확인하는 동안에만 `infer_hz`로 돈다. 주행(fleet_agent)을 막지 않는다
- 켜기: `ros2 launch fleet_agent robot_system.launch.xml ... use_guide:=true`
  - 모델: 저장소 `models/best.pt`(yolov8n, 클래스 toy_person / toy_car / cone)가 빌드 때 `share/fleet_agent/models/`에 같이 설치되고 기본값으로 쓰인다.
    손님 = `class_name: toy_person` (config/fleet_agent.yaml guide 쪽)
  - 다른 모델: `guide_model:=<경로.pt>` (클래스 이름이 다르면 class_name도), 모델 없이 흐름만: `guide_model:=""`
  - 카메라 토픽 `camera_topic`(기본 /camera/image_raw)을 내는 노드가 따로 있어야 한다
  - guide 노드를 안 켜면 손님 확인을 건너뛰고 바로 진행, 모델이 없으면 guide 노드가 바로 OK (흐름 시험용)
- 필요: `pip install ultralytics` (로봇 파이썬 환경)

## 노드 순회 시험 (node_tour) - 서버 없이 Nav2 node to node 확인

서버(main_service)는 끄고, 로봇에서 `frun N1 -1.57 360`으로 켠 뒤 다른 터미널에서:
```bash
ros2 run fleet_agent node_tour                     # 기본: 홀 N1 -> N13 지그재그 -> N1 (T 노드도 지나감)
ros2 run fleet_agent node_tour N1 N4 N13 N11 N1    # 목표 노드만 적으면 사이 노드는 길 찾기로 채움
ros2 run fleet_agent node_tour --step 2 N1 N4 N1   # 목표마다 멈춰 2초 쉬고
ros2 run fleet_agent node_tour --loops 3
```
노드마다 걸린 시간 · 도착 때 위치 오차(TF vs 노드 좌표)를 찍고, 끝나면 표로 정리한다. Ctrl+C = 멈추고 결과 출력.
후진 구간 · 도킹 · 픽업은 목표로 적지 않으면 안 지나간다 (전진 주행만).

## 실제 로봇 시험 (정밀 정차 끔)

지금은 도킹(마커 · 라인)을 빼고 **주문 → P1에서 5초 대기 → 테이블 → 도킹 자리로 복귀**만 한다.
서버 설정 `main_service.yaml`: `auto_ready: true`(주방 버튼 없이 바로 출발), `pickup_timeout_sec: 5`(적재 버튼 없이 5초 뒤 출발)

```bash
# ---- 로봇 (pinky1, ROS_DOMAIN_ID=1) : 도킹 자리(예: D1)에 올려 두고
cd ~/ros-repo-2 && colcon build --packages-up-to fleet_agent && source install/setup.bash
# 1) bringup (모터 · LiDAR · odom)    2) Nav2 + 맵 project_map_v2, RViz로 초기 위치를 D1에 맞춤
ros2 launch fleet_agent fleet_agent.launch.xml robot_id:=pinky1 start_node:=D1

# ---- 서버 노트북 (ROS_DOMAIN_ID=0)
cd ~/ros-repo-2 && colcon build --packages-up-to pinkytorang_main && source install/setup.bash
ros2 launch src/fleet_msgs/bridge/fleet_bridge.launch.xml pinky2:=false pinky3:=false
ros2 run pinkytorang_main main_service src/main_service/config/main_service.yaml
# 브라우저: http://<노트북 IP>:8080/c?table=T1 에서 주문 (매장 화면 /store)
```

확인할 로그 (fleet_agent 터미널)
```
Move cmd ... REPLACE FORWARD ['N1', 'N2']     ← 서버 명령이 들어옴
event NODE_REACHED N1 / N2 ...                 ← 노드마다
Move cmd ... APPEND REVERSE ['N15']            ← N14에서 돌아서 후진 시작
event CMD_DONE P1                              ← 픽업 도착 (5초 뒤 서버가 테이블로 보냄)
event CMD_DONE T1                              ← 테이블 도착 (수령 버튼 또는 30초 뒤 도킹 자리로)
```
- 서버가 켜질 때 **쉬는 로봇 자리를 맞춘다** (빈 테이블이 많으면 입구 D1 · D2에 2대). 1대만 시험하면 D1에서 켜는 게 편하다.
  켜자마자 움직이는 게 싫으면 서버 설정 `entrance_standby_quiet: 0`, `entrance_standby_busy: 0`
- 처음에는 `reverse_speed`(0.06 m/s)와 Nav2 속도를 낮게 두고 본다

## 빌드 · 실행

```bash
cd ~/ros-repo-2 && colcon build --packages-select fleet_msgs fleet_agent && source install/setup.bash

# 로봇마다 (ROS_DOMAIN_ID = 1 / 2 / 3). Nav2 · 맵(project_map_v2) · 카메라 · LiDAR는 먼저 띄워 둔다
ros2 launch fleet_agent fleet_agent.launch.xml robot_id:=pinky1 start_node:=D1
```

- `start_node`: 켤 때 서 있는 도킹. 서버는 처음 `last_node`가 도킹이어야 배정한다 (`require_dock_start`)
- 설정: `config/fleet_agent.yaml` (속도 · 허용 오차 · 장애물 거리 · 대기 시간)
- 정밀 정차 노드: `fleet_msgs/docs/precision_stops.yaml` (launch가 자동으로 읽음)
- 로직 테스트 (ROS 없이, 가짜 로봇): `python3 test/test_core.py`

## 로봇에 맞춰 확인할 것

- `/cmd_vel`, `/scan`, `navigate_to_pose` 이름과 TF `map -> base_link`
- `obstacle`의 `scan_yaw_offset` (LiDAR 0도가 로봇 앞인지), 앞 · 뒤 멈춤 거리
- 배터리 토픽 (`battery_topic`, 없으면 `battery_default` 100%)
- LCD 드라이버가 `/lcd/text`를 받게
