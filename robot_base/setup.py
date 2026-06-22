from setuptools import setup

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_base_launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml', 'config/slam_toolbox.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dzawkan',
    maintainer_email='you@example.com',
    description='Serial bridge antara Arduino dan ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = robot_base.serial_bridge_node:main',
            'rotate_calibration = robot_base.rotate_calibration:main',
            'lidar_node = robot_base.lidar_node:main',
            'translate_calibration = robot_base.translate_calibration:main',
        ],
    },
)