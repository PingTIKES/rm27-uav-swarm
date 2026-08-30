"""OpenVINS 启动（实机用）。仿真中不需要本文件。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav_ns_arg = DeclareLaunchArgument('uav_ns', default_value='uav1')
    params = os.path.join(
        get_package_share_directory('uav_localization'),
        'config', 'openvins_params.yaml')

    return LaunchDescription([
        uav_ns_arg,
        Node(
            package='ov_msckf',          # third_party/open_vins 编译后提供
            executable='run_subscribe_msckf',
            name='openvins',
            namespace=LaunchConfiguration('uav_ns'),
            parameters=[params],
            output='screen',
        ),
        # 视觉里程计转飞控：/uavN/odom -> /px4_N/fmu/in/vehicle_visual_odometry
        # 实机在此加一个转换节点（注意 ENU<->NED 与机体坐标变换），
        # 并设置飞控参数 EKF2_EV_CTRL=15 启用视觉融合。
    ])
