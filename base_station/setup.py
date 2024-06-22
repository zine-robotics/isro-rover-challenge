from setuptools import setup
import os
from glob import glob

package_name = 'base_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
          (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
          (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Priyansh',
    maintainer_email='345priyansh@gmail.todo',
    description='The hardware controlelrs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_station_gui = base_station.mission_planner_gui:main',
        ],
    },
)
