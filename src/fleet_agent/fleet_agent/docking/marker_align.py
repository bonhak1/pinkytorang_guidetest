#!/usr/bin/env python3
"""방향 정렬 기준 저장 · 시험 (undock의 align 단계가 이 파일을 읽음)

마커의 기울기(각도) 대신 '방위각'(로봇 중심에서 본 마커 방향)만 쓴다. 멀어도(50 cm) 0.1° 수준으로 안정적.
제자리 회전만 하고 앞뒤 · 좌우로는 안 움직인다.

  기준 저장 (N15에서 언도킹 + 왼쪽 90도 돈 자세, 반듯한지 눈으로 확인하고):
    ros2 run fleet_agent marker_align --id 0 --size 0.05 --name n14 --record    -> ~/align_n14.json
  확인만:   ros2 run fleet_agent marker_align --name n14 --dry-run
  정렬:     ros2 run fleet_agent marker_align --name n14
(fleet_agent를 켜 두면 카메라를 docking 노드가 쥐고 있으니 이건 끄고 한다)
"""
import argparse
import json
import math
import os

import numpy as np

from fleet_agent.docking import marker_dock as md


def bearing(m):
    return math.atan2(-m['tx'], md.CAM_X_M + m['tz'])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.signals import SignalHandlerOptions
    from tf2_ros import Buffer, TransformListener

    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--id', type=int, help='마커 ID (저장할 때)')
    ap.add_argument('--size', type=float, default=0.025, help='마커 한 변 m (저장할 때, 5 cm = 0.05)')
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--direct', action='store_true', help='camera_node 없이 카메라를 직접 엶')
    args, _ = ap.parse_known_args()
    path = os.path.expanduser(f'~/align_{args.name}.json')
    md.load_cam_file()
    if args.record:
        if args.id is None:
            raise SystemExit('--record 에는 --id 가 필요합니다')
        mid, size, ref = args.id, args.size, None
    else:
        with open(path) as f:
            ref = json.load(f)
        mid, size = ref['id'], ref.get('size', 0.025)
    md.MARKER_SIZE_M = size
    h = size / 2
    md.OBJ = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('marker_align')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buf = Buffer()
    tf_listener = TransformListener(tf_buf, node)  # noqa: F841
    cam = md.Camera(mid) if getattr(args, 'direct', False) else md.Camera(mid, node, spin=True)   # 기본: camera_node 구독
    dk = md.Docker(node, pub, tf_buf, cam, moving=not (args.record or args.dry_run), spin=True)
    result = None
    try:
        dk.wait_pose()
        if args.record:
            m = cam.measure_stable(frames=30)
            if m is None:
                result = f'실패: 마커 {mid}가 안 보임'
                return
            with open(path, 'w') as f:
                json.dump({'id': mid, 'size': size, 'bearing': bearing(m), 'tz': m['tz']}, f, indent=2)
            result = f'저장: 방위각 {math.degrees(bearing(m)):+.2f}°, 거리 {m["tz"] * 100:.1f} cm -> {path}'
            return
        for k in range(1, 4):
            dk.stop(0.12)
            m = cam.measure_stable(frames=6)
            if m is None:
                result = f'실패: 마커 {mid}가 안 보임'
                return
            err = bearing(m) - ref['bearing']
            print(f'[{k}] 방향 오차 {math.degrees(err):+.2f}°')
            if abs(math.degrees(err)) < 0.5 or args.dry_run:
                break
            dk.rotate_to(md.wrap(dk.pose()[2] + err))
        result = '성공'
    except KeyboardInterrupt:
        result = '중단: Ctrl+C'
    except Exception as e:
        result = f'실패: {e}'
    finally:
        try:
            dk.stop(0.1)
        except Exception:
            pass
        cam.close()
        print(f'결과: {result}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
