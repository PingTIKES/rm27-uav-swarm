"""
实机单机启动（RK3566 机载端，对应框架文档 6.1 节）。

前提：
  1. 飞控已通过串口接入：MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600
  2. 双目相机驱动已发布 /uavN/camera/{left,right}/image_raw
  3. OpenVINS 已启动（ros2 launch uav_localization openvins.launch.py）

用法：ros2 launch uav_bringup uav_bringup.launch.py uav_id:=1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav_id = LaunchConfiguration('uav_id')
    params_file = os.path.join(
        get_package_share_directory('uav_bringup'), 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('uav_id', default_value='1'),

        # 飞控 Offboard 桥（实机建议 auto_takeoff:=False，先遥控器接管验证）
        Node(
            package='uav_control',
            executable='offboard_control',
            namespace=['uav', uav_id],
            parameters=[{
                'px4_ns': ['px4_', uav_id],
                'px4_instance': uav_id,
                'takeoff_alt': 1.5,
                'auto_takeoff': False,     # 实机首飞务必改为 False！
            }],
            output='screen',
        ),
        # YOLO 目标检测（RK3566 NPU）
        Node(
            package='uav_perception',
            executable='yolo_detector',
            namespace=['uav', uav_id],
            output='screen',
        ),
        # 集群调度（可放在地面站，也可由长机携带）
        Node(
            package='uav_swarm',
            executable='swarm_coordinator',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='uav_planning',
            executable='collision_monitor',
            parameters=[params_file],
            output='screen',
        ),
    ])
