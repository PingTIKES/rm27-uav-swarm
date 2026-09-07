from setuptools import setup

package_name = 'uav_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', ['maps/rmuc_2025_occ.npz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm27-uav',
    maintainer_email='team@example.com',
    description='规划与防碰节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collision_monitor = uav_planning.collision_monitor:main',
            'vfh_planner = uav_planning.vfh_planner:main',
            'goal_planner = uav_planning.goal_planner:main',
            'pose_tf_publisher = uav_planning.pose_tf_publisher:main',
        ],
    },
)
