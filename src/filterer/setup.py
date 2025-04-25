from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'filterer'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdger',
    maintainer_email='okhere21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    package_data={'goal_calculator': ['action/*.action']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'lane_filter_node = filterer.lane_filterer:main',
        ],
    },
)