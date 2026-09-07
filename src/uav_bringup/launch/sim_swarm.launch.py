"""
四机集群仿真一键启动（需先运行 scripts/start_sim_4uav.sh）。

用法：
    ros2 launch uav_bringup sim_swarm.launch.py num_uavs:=4

启动内容：
    每机：uav_control/offboard_control（命名空间 uavN，桥接 px4_N）
          uav_perception/sim_target_detector（仿真目标检测）
    集群：uav_swarm/swarm_coordinator（分区搜索/汇聚/返航）
          uav_planning/collision_monitor（机间防碰监视）

出生点：SPAWN_OFFSETS_NED 必须与 scripts/start_sim_4uav.sh 的
SPAWN_POSES（ENU）对应，换算关系 NED = (enu_y, enu_x)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 各机出生点（公共系 NED）：蓝方基地启动区四周
SPAWN_OFFSETS_NED = [
    [9.4, 1.3],    # uav1 <- ENU "1.3,9.4"
    [9.4, -1.3],   # uav2 <- ENU "-1.3,9.4"
    [11.6, 1.3],   # uav3 <- ENU "1.3,11.6"
    [11.6, -1.3],  # uav4 <- ENU "-1.3,11.6"
]


def _setup(context, *args, **kwargs):
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))
    spacing = float(LaunchConfiguration('spawn_spacing').perform(context))
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'

    params_file = os.path.join(
        get_package_share_directory('uav_bringup'), 'config', 'params.yaml')

    def offset_of(i):
        if i <= len(SPAWN_OFFSETS_NED):
            return SPAWN_OFFSETS_NED[i - 1]
        return [0.0, (i - 1) * spacing]  # 超出 4 机时一字排开兜底

    nodes = []
    for i in range(1, num_uavs + 1):
        # 飞控 Offboard 桥（每机一个，命名空间 uavN）
        nodes.append(Node(
            package='uav_control',
            executable='offboard_control',
            name='offboard_control',
            namespace=f'uav{i}',
            parameters=[{
                'px4_ns': f'px4_{i}',
                'px4_instance': i,
                'takeoff_alt': 2.0 + (i - 1) * 0.5,   # 与集群高度层一致
                'auto_takeoff': True,
            }],
            output='screen',
        ))
        # 仿真目标检测器（实机换成 yolo_detector）
        if sim:
            nodes.append(Node(
                package='uav_perception',
                executable='sim_target_detector',
                name='sim_target_detector',
                namespace=f'uav{i}',
                parameters=[params_file, {
                    'px4_ns': f'px4_{i}',
                    'spawn_offset': offset_of(i),
                }],
                output='screen',
            ))

    # 集群调度
    nodes.append(Node(
        package='uav_swarm',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        parameters=[params_file, {'num_uavs': num_uavs, 'spawn_spacing': spacing}],
        output='screen',
    ))
    # 机间防碰监视
    nodes.append(Node(
        package='uav_planning',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[params_file, {'num_uavs': num_uavs, 'spawn_spacing': spacing}],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='4'),
        DeclareLaunchArgument('spawn_spacing', default_value='2.0'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='true=仿真检测器；false=实机 YOLO'),
        OpaqueFunction(function=_setup),
    ])
