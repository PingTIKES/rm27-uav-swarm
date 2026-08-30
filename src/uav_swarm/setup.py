from setuptools import setup

package_name = 'uav_swarm'

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
    description='四机集群调度节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_coordinator = uav_swarm.swarm_coordinator:main',
        ],
    },
)
