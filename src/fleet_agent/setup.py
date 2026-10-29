from glob import glob

from setuptools import setup

package_name = 'fleet_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.docking'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.pt')),   # 안내 손님 확인 YOLO (best.pt)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='todo@example.com',
    description='로봇 쪽 fleet 에이전트',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_agent = fleet_agent.agent_node:main',
            'docking_node = fleet_agent.docking_node:main',
            'camera_node = fleet_agent.camera_node:main',       # 카메라 혼자 열고 /camera/image_raw 발행
            'obstacle_node = fleet_agent.obstacle_node:main',
            'initial_pose = fleet_agent.initial_pose:main',
            'node_tour = fleet_agent.node_tour:main',
            'guide_node = fleet_agent.guide_node:main',         # 안내: 카메라 YOLO 손님 확인 · 효과
            # ---- 안내 시험 (GUIDE_TEST.md)
            'guide_run = fleet_agent.guide_run:main',           # 서버 없이 안내 주행 (실제 로봇)
            'guide_sim = fleet_agent.guide_sim:main',           # PC 시뮬레이터 (ROS · 로봇 없이 Driver)
            'guide_check = fleet_agent.guide_check:main',       # guide_node 단독 (greet / watch / arrive / 취소)
            'fake_guide = fleet_agent.fake_guide:main',         # 카메라 없이 손님 결과를 정하는 가짜 guide
            'detect_view = fleet_agent.detect_view:main',       # YOLO 인식 튜닝 (conf · 거리)
            'guide_effect = fleet_agent.guide_effects:main',    # LCD · LED · 부저 효과만
            'marker_dock = fleet_agent.docking.marker_dock:main',   # 혼자 실행: --calibrate / --record / --dry-run
            'marker_align = fleet_agent.docking.marker_align:main', # 혼자 실행: 정렬 기준 --record (undock의 align)
        ],
    },
)
