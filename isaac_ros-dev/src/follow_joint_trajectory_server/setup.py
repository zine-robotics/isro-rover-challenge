from setuptools import setup

package_name = 'follow_joint_trajectory_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trajectory_action_server.launch.py']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aswin',
    maintainer_email='aswinarunkumar8@gmail.todo',
    description='The arm action server',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_action_server = follow_joint_trajectory_server.trajectory_action_server:main',
            'arm_serial_arduino_node = follow_joint_trajectory_server.serial_arduino_node:main',
            'joint_state_publisher=follow_joint_trajectory_server.joint_state_publisher:main'
        ],
    },
)
