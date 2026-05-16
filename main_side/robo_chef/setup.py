from setuptools import find_packages, setup

package_name = 'robo_chef'

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
    maintainer='kibeom',
    maintainer_email='neopkrrl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'executer = nodes.executer:main',
            'firbase_bridge = nodes.firebase_bridge:main',
            'recipe_parser = nodes.recipe_parser:main',
            'recipe_tester = nodes.recipe_tester:main',
            'state_manager = nodes.state_manager:main',
        ],
    },
)
