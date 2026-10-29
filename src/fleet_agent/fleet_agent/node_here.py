"""지금 로봇 위치를 노드 좌표로 기록: node_graph.yaml의 그 노드 x, y, yaw를 실제 값으로 바꾼다

로봇을 그 자리(예: 입구 D1)에 실제 방향대로 놓고, 맵 위치가 맞게 잡힌 뒤(RViz 2D Pose Estimate) 실행한다.
TF(map -> base_link)만 보면 되므로 로봇과 같은 ROS_DOMAIN_ID인 PC에서 돌려도 된다.

  ros2 run fleet_agent node_here D1              # 지금 위치 · 방향만 출력
  ros2 run fleet_agent node_here D1 --write      # node_graph.yaml의 D1을 고침 (주석 · 다른 줄은 그대로)
  ros2 run fleet_agent node_here D1 --write --graph ~/project/pinkytorang_guidetest/src/fleet_msgs/docs/node_graph.yaml

고친 뒤: 레포에서 fleet_msgs 다시 빌드 (서버 · 로봇 둘 다 같은 node_graph를 써야 함) -> commit · push
"""
import argparse
import datetime
import math
import os
import re
import sys

CANDIDATES = ('src/fleet_msgs/docs/node_graph.yaml',
              '~/project/pinkytorang_guidetest/src/fleet_msgs/docs/node_graph.yaml',
              '~/pinkytorang_guidetest/src/fleet_msgs/docs/node_graph.yaml')


def find_graph():
    for c in CANDIDATES:
        p = os.path.abspath(os.path.expanduser(c))
        if os.path.exists(p):
            return p
    return ''


def update_node_line(text, node, x, y, yaw, note=''):
    """node_graph.yaml 글에서 `- {id: <node>, ...}` 한 줄의 x, y, yaw만 바꾼다 -> (새 글, 옛 줄, 새 줄)"""
    pat = re.compile(r'^(\s*-\s*\{\s*id:\s*' + re.escape(node) + r'\s*,.*)$', re.M)
    m = pat.search(text)
    if not m:
        raise KeyError(f'node_graph에 {node} 줄이 없음')
    old = m.group(1)
    body, sep, comment = old.partition('}')
    body = re.sub(r'(\bx:\s*)-?[\d.]+', lambda k: f'{k.group(1)}{x:.3f}', body, count=1)
    body = re.sub(r'(\by:\s*)-?[\d.]+', lambda k: f'{k.group(1)}{y:.3f}', body, count=1)
    if re.search(r'\byaw:\s*-?[\d.]+', body):
        body = re.sub(r'(\byaw:\s*)-?[\d.]+', lambda k: f'{k.group(1)}{yaw:.3f}', body, count=1)
    else:
        body = re.sub(r'(\by:\s*-?[\d.]+)', lambda k: f'{k.group(1)}, yaw: {yaw:.3f}', body, count=1)
    comment = comment.replace('[추정] ', '').replace('[추정]', '')
    if note:
        comment = comment.rstrip() + f'  [{note}]' if comment.strip() else f'  # [{note}]'
    new = body + sep + comment
    return text[:m.start(1)] + new + text[m.end(1):], old, new


def main():
    from rclpy.utilities import remove_ros_args
    ap = argparse.ArgumentParser(description='지금 로봇 위치를 node_graph 노드 좌표로 기록')
    ap.add_argument('node', help='예: D1')
    ap.add_argument('--write', action='store_true', help='node_graph.yaml을 고침')
    ap.add_argument('--graph', default='', help='node_graph.yaml (기본: 레포 src/fleet_msgs/docs)')
    ap.add_argument('--samples', type=int, default=20, help='TF를 이만큼 받아 평균')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    import time

    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('node_here')
    tf = Buffer()
    TransformListener(tf, node)
    xs, ys, ss, cs = [], [], [], []
    end = time.monotonic() + 10.0
    while len(xs) < a.samples and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = tf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.0))
        except Exception:  # noqa: BLE001
            continue
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        xs.append(t.transform.translation.x)
        ys.append(t.transform.translation.y)
        ss.append(math.sin(yaw))
        cs.append(math.cos(yaw))
    node.destroy_node()
    rclpy.shutdown()
    if not xs:
        sys.exit('TF map -> base_link를 못 받음: 로봇 Nav2(AMCL)가 떠 있고 ROS_DOMAIN_ID가 같은지 확인')
    x, y = sum(xs) / len(xs), sum(ys) / len(ys)
    yaw = math.atan2(sum(ss), sum(cs))
    spread = max(math.hypot(px - x, py - y) for px, py in zip(xs, ys))
    print(f'{a.node}: x {x:.3f}  y {y:.3f}  yaw {yaw:.3f} rad ({math.degrees(yaw):.0f}°)   '
          f'(TF {len(xs)}번, 흔들림 {spread * 100:.1f} cm)')
    if spread > 0.03:
        print('⚠ 위치가 흔들림: 로봇이 멈춰 있고 AMCL이 자리 잡은 뒤 다시')
    if not a.write:
        print('node_graph에 쓰려면 --write')
        return
    path = os.path.expanduser(a.graph) if a.graph else find_graph()
    if not path or not os.path.exists(path):
        sys.exit('node_graph.yaml을 못 찾음: --graph로 지정 (레포 src/fleet_msgs/docs/node_graph.yaml)')
    with open(path, encoding='utf-8') as f:
        text = f.read()
    note = '실측 ' + datetime.date.today().isoformat()
    new_text, old, new = update_node_line(text, a.node, x, y, yaw, note)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(new_text)
    print(f'고침 {path}\n  전: {old.strip()}\n  후: {new.strip()}')
    print('다음: colcon build --packages-up-to fleet_agent main_service  (서버 · 로봇 둘 다), 그리고 commit · push')


if __name__ == '__main__':
    main()
