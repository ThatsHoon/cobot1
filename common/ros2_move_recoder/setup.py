from setuptools import find_packages, setup

package_name = 'ros2_move_recoder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoon',
    maintainer_email='gagea45@gmail.com',
    description='Doosan m0609 매크로 기록/평활화/재생 (PyQt5 GUI)',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'run = ros2_move_recoder.run:main',
            'recorder = ros2_move_recoder.recorder:main',
            'smoother = ros2_move_recoder.smoother:main',
            'player = ros2_move_recoder.player:main',
            'gui = ros2_move_recoder.gui:main',
        ],
    },
)
