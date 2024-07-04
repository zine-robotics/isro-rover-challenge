from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_stage_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zine',
    maintainer_email='zine@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tree_node=rover_stage_manager.tree_node:main',
            'rover_server=rover_stage_manager.rover_server:main',
        ],
    },
)
