from setuptools import find_packages, setup

package_name = 'action_servers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raman',
    maintainer_email='raman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mandatory_wpf = action_servers.mandatory_wpf:main',
            'pickup_object = action_servers.pickup_object:main',
            'rupesh = action_servers.rupesh:main',
            'calibration = action_servers.calibration:main',

        ],
    },
)
