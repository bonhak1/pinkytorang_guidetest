"""Guide 노드 단독 시험: fleet_agent · Nav2 없이 /guide/request를 직접 보내고 /guide/result를 잰다

로봇은 가만히 둔다 (안 움직인다). 카메라 앞에 손님 인형을 두거나 치우면서 guide_node가 제대로 답하는지 본다.

  ros2 run fleet_agent guide_check greet               # 인사: 손님이 보이면 OK + 환영 효과
  ros2 run fleet_agent guide_check watch               # 확인: 손님이 보이는지
  ros2 run fleet_agent guide_check arrive              # 도착 효과만 (결과 없음)
  ros2 run fleet_agent guide_check watch --repeat 5    # 5번 연달아 (인형을 놓았다 치웠다 하며)
  ros2 run fleet_agent guide_check watch --cancel 3    # 3초 뒤 취소 -> 결과가 안 와야 정상
  ros2 run fleet_agent guide_check all                 # greet -> watch -> arrive -> 취소 차례로 (점검표)

guide_node는 먼저 띄워 둔다 (둘 중 하나)
  ros2 run fleet_agent guide_node --ros-args --params-file <fleet_agent.yaml> -p model_path:=<best.pt>
  ros2 launch fleet_agent fleet_agent.launch.xml use_guide:=true     (fleet_agent도 같이 뜸. 움직이지는 않음)
"""
import argparse
import json
import sys
import time

TIMEOUT_PAD = 5.0


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    ap = argparse.ArgumentParser(description='Guide 노드 단독 시험 (/guide/request -> /guide/result)')
    ap.add_argument('action', choices=['greet', 'watch', 'arrive', 'all'])
    ap.add_argument('--node', default='TEST', help='요청에 넣을 노드 이름')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--gap', type=float, default=2.0, help='반복 사이 쉬는 초')
    ap.add_argument('--cancel', type=float, default=0.0, help='이 초 뒤 취소 (0 = 안 함)')
    ap.add_argument('--timeout', type=float, default=30.0, help='guide wait_sec (이만큼 + 5초 기다림)')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('guide_check')
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    req_pub = node.create_publisher(String, '/guide/request', rel)
    cancel_pub = node.create_publisher(String, '/guide/cancel', rel)
    results = []
    node.create_subscription(String, '/guide/result', lambda m: results.append((time.monotonic(), m.data)), rel)

    # guide 노드가 구독할 때까지 (최대 5초)
    t_end = time.monotonic() + 5.0
    while req_pub.get_subscription_count() == 0 and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if req_pub.get_subscription_count() == 0:
        print('❌ /guide/request 구독자가 없음: guide_node(또는 fake_guide)가 안 떠 있다 (ROS_DOMAIN_ID도 확인)')
        rclpy.shutdown()
        return 1
    time.sleep(0.3)                      # 결과 토픽 연결

    rid = int(time.time()) % 100000 * 10
    rows = []

    def one(action, cancel_after=0.0):
        nonlocal rid
        rid += 1
        results.clear()
        t0 = time.monotonic()
        req_pub.publish(String(data=json.dumps({'id': rid, 'action': action, 'node': a.node})))
        print(f'→ {action} (id {rid}){f", {cancel_after}초 뒤 취소" if cancel_after else ""} ...', flush=True)
        if action == 'arrive':
            print('  도착 효과가 나와야 함 (LCD ARRIVED! · LED · 부저). 결과는 안 옴')
            rows.append((action, '-', 0.0, 'effect only'))
            return
        wait = (cancel_after + 2.0) if cancel_after else a.timeout + TIMEOUT_PAD
        cancelled = False
        while time.monotonic() - t0 < wait:
            rclpy.spin_once(node, timeout_sec=0.05)
            if cancel_after and not cancelled and time.monotonic() - t0 >= cancel_after:
                cancel_pub.publish(String(data=json.dumps({'id': rid})))
                cancelled = True
            mine = [(t, d) for t, d in results if json.loads(d).get('id') == rid]
            if mine:
                t, d = mine[0]
                r = json.loads(d)
                verdict = 'OK' if r.get('ok') else '못 찾음'
                if cancelled:
                    verdict += ' (취소했는데 결과가 옴 ❌)'
                print(f'  결과 {verdict}  detail={r.get("detail")}  {t - t0:.2f}초')
                rows.append((action, verdict, t - t0, r.get('detail', '')))
                return
        if cancelled:
            print('  결과 없음 ✅ (취소되면 안 오는 게 정상)')
            rows.append((action + '+cancel', '결과 없음', time.monotonic() - t0, 'cancel ok'))
        else:
            print('  결과 없음 ❌ (시간 초과: guide_node 로그 확인)')
            rows.append((action, '시간 초과', time.monotonic() - t0, ''))

    try:
        if a.action == 'all':
            input('1) 카메라 앞에 손님 인형을 두고 Enter (greet) ')
            one('greet')
            input('2) 인형을 치우고 Enter (watch: 못 찾음이 나와야 함, 최대 wait_sec) ')
            one('watch')
            input('3) Enter (arrive 효과) ')
            one('arrive')
            time.sleep(2.0)
            input('4) Enter (watch 시작 2초 뒤 취소: 결과가 없어야 함) ')
            one('watch', 2.0)
        else:
            for i in range(max(1, a.repeat)):
                if i:
                    time.sleep(a.gap)
                one(a.action, a.cancel)
    except KeyboardInterrupt:
        pass

    print('\n==== guide_check ====')
    for action, verdict, sec, detail in rows:
        print(f'{action:13s} {verdict:22s} {sec:6.2f}초  {detail}')
    print('detail: guest_seen = YOLO로 찾음 / not_seen = 못 찾음 / no_camera = 영상이 안 옴 / no_detector = 모델 없음(바로 OK)')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
