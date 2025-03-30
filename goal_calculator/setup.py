from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'goal_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include action files
        (os.path.join('share', 'action'), glob('action/*.action')),
        (os.path.join('share', 'msg'), glob('msg/*.msg')),
        (os.path.join('share', 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdger',
    maintainer_email='janak3.1415et@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    package_data={'goal_calculator': ['action/*.action']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'lane_follow = goal_calculator.lane_follow:main',
            'lane_change = goal_calculator.lane_change:main',
            'right_turn = goal_calculator.right_turn:main',
            'left_turn = goal_calculator.left_turn:main'
        ],
    },
)
