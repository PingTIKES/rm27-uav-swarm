from setuptools import setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm27-uav',
    maintainer_email='team@example.com',
    description='感知节点（仿真检测器 + YOLO 桩）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_target_detector = uav_perception.sim_target_detector:main',
            'yolo_detector = uav_perception.yolo_detector:main',
        ],
    },
)
