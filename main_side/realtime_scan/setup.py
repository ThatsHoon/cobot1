from setuptools import find_packages, setup

package_name = 'realtime_scan'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahn',
    maintainer_email='gagea45@gmail.com',
    description='ROBO CHEF — Realtime Coord Scan & Order Receiver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # ros2 run realtime_scan coord_service
            'coord_service = realtime_scan.coord_service:main',
            # ros2 run realtime_scan order_receiver
            'order_receiver = realtime_scan.order_receiver:main',
            # ros2 run realtime_scan log_bridge  (rosout → Firebase /dsr_log)
            'log_bridge = realtime_scan.log_bridge:main',
        ],
    },
)
