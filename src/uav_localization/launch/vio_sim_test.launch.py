"""OpenVINS 仿真测试链路：ros_gz_bridge（Gazebo Garden -> ROS2）+ ov_msckf。

话题映射：
  gz /world/<world>/model/<vio_model>/link/base_link/sensor/imu_sensor/imu
      -> /<uav_ns>/imu0          (sensor_msgs/Imu, 250Hz, 与 PX4 同源)
  gz /vio_cam0/image -> /<uav_ns>/cam0/image_raw  (sensor_msgs/Image mono8, 30Hz)
  gz /vio_cam1/image -> /<uav_ns>/cam1/image_raw

OpenVINS 输出（namespace <uav_ns> 下）：
  odomimu / pathimu / points_msckf / trackhist（特征跟踪可视化图）

前置条件：
  1) 仿真以 VIO 模式启动：VIO_UAV=1 ./scripts/start_sim_4uav.sh
  2) 本进程环境与仿真同一 GZ_PARTITION（run_openvins_sim.sh 已自动处理）
  3) 已安装 ros_gz_bridge（Humble+Garden 用 OSRF 源的 ros-humble-ros-gzgarden）
  4) 已编译并 source OpenVINS（ov_msckf 包）

一般由 scripts/run_openvins_sim.sh 调用，也可手动：
  source /tmp/rm27_gz_env.sh
  ros2 launch uav_localization vio_sim_test.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('vio_model').perform(context)
    ns = LaunchConfiguration('uav_ns').perform(context)
    config = LaunchConfiguration('config').perform(context)
    with_ov = LaunchConfiguration('with_openvins').perform(context).lower() \
        in ('true', '1', 'yes')

    imu_gz = (f'/world/{world}/model/{model}'
              f'/link/base_link/sensor/imu_sensor/imu')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vio_gz_bridge',
        output='screen',
        arguments=[
            f'{imu_gz}@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/vio_cam0/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/vio_cam1/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            (imu_gz, f'/{ns}/imu0'),
            ('/vio_cam0/image', f'/{ns}/cam0/image_raw'),
            ('/vio_cam1/image', f'/{ns}/cam1/image_raw'),
        ],
    )
    nodes = [bridge]

    if with_ov:
        nodes.append(Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='openvins',
            namespace=ns,
            output='screen',
            parameters=[{
                'config_path': config,
                'topic_imu': f'/{ns}/imu0',
                'topic_camera0': f'/{ns}/cam0/image_raw',
                'topic_camera1': f'/{ns}/cam1/image_raw',
                'publish_global_to_imu_tf': True,
                'publish_calibration_tf': True,
            }],
        ))
    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('uav_localization'),
        'config', 'openvins_sim', 'estimator_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rmuc_2025_field',
                              description='Gazebo 世界名'),
        DeclareLaunchArgument('vio_model', default_value='x500_stereo_1',
                              description='世界中带双目的模型实例名'),
        DeclareLaunchArgument('uav_ns', default_value='uav1',
                              description='ROS 侧话题命名空间'),
        DeclareLaunchArgument('config', default_value=default_config,
                              description='OpenVINS estimator_config.yaml 路径'),
        DeclareLaunchArgument('with_openvins', default_value='true',
                              description='false 时只起桥接（用于先验证数据通路）'),
        OpaqueFunction(function=_setup),
    ])
