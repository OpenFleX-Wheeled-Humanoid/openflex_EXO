#!/usr/bin/env python3
from glob import glob
from setuptools import find_packages, setup

package_name = 'openflex_teleop_exo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chengdu Changshu Robot Co., Ltd',
    maintainer_email='openarmrobot@gmail.com',
    description='EXO whole-body teleoperation adapter for OpenFlex.',
    license='OpenArmX Research and Education License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_teleoperator = openflex_teleop_exo.websocket_teleoperator:main',
            'openflex_exo_teleop_node = openflex_teleop_exo.openflex_exo_teleop_node:main',
            'openflex_arm_retargeting_node = openflex_teleop_exo.arm_retargeting_node:main',
            'openflex_arm_bridge_node = openflex_teleop_exo.arm_bridge_node:main',
        ],
    },
)
