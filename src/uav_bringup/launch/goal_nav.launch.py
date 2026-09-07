"""
RViz 打点导航一键启动（需先运行 scripts/start_sim_4uav.sh）。

用法：
    ros2 launch uav_bringup goal_nav.launch.py              # 打点控制 uav1
    ros2 launch uav_bringup goal_nav.launch.py uav_id:=3    # 打点控制 uav3

启动内容：
    每机：uav_control/offboard_control（4 机自动起飞到分层高度悬停）
    规划：uav_planning/goal_planner（订阅 /goal_pose，A* 规划后逐航点下发）
    显示：uav_planning/pose_tf_publisher（map->uavN TF + 机身标记）
          rviz2（加载 config/rm2025.rviz，含场地地图/路径/标记）

操作：RViz 顶部工具栏点 "2D Nav Goal"，在地图上按住拖出目标点与朝向，
被控无人机即沿规划路径（绿色线）飞往该点；其余机原地悬停。

注意：本 launch 与 sim_swarm.launch.py 二选一（两者都会给 /uavN/waypoint
发航点，同时跑会互相抢控制权）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))
    uav_id = int(LaunchConfiguration('uav_id').perform(context))

    share = get_package_share_directory('uav_bringup')
    params_file = os.path.join(share, 'config', 'params.yaml')
    rviz_config = os.path.join(share, 'config', 'rm2025.rviz')

    nodes = []
    for i in range(1, num_uavs + 1):
        nodes.append(Node(
            package='uav_control',
            executable='offboard_control',
            name='offboard_control',
            namespace=f'uav{i}',
            parameters=[{
                'px4_ns': f'px4_{i}',
                'px4_instance': i,
                'takeoff_alt': 2.0 + (i - 1) * 0.5,
                'auto_takeoff': True,
            }],
            output='screen',
        ))

    # 打点路径规划
    nodes.append(Node(
        package='uav_planning',
        executable='goal_planner',
        name='goal_planner',
        parameters=[params_file, {
            'uav_id': uav_id,
            # 巡航高度与被控机的起飞高度一致（uav_id=1 -> 2.0 m）
            'cruise_alt': 2.0 + (uav_id - 1) * 0.5,
        }],
        output='screen',
    ))
    # 位姿 -> TF + 标记
    nodes.append(Node(
        package='uav_planning',
        executable='pose_tf_publisher',
        name='pose_tf_publisher',
        parameters=[params_file, {'num_uavs': num_uavs}],
        output='screen',
    ))
    # RViz
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='4'),
        DeclareLaunchArgument('uav_id', default_value='1',
                              description='2D Nav Goal 控制哪台无人机'),
        OpaqueFunction(function=_setup),
    ])
