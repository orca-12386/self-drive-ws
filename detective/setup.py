from setuptools import find_packages, setup

package_name = 'detective'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/trained_models/', ['trained_models/Drums.pt','trained_models/Pedestrian.pt','trained_models/StopSigns.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='summer',
    maintainer_email='okhere21@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drum_detector_node = detective.drum_detector_node:main',
            'stop_sign_detector_node = detective.stop_sign_detector_node:main',
            'pedestrian_detector_node = detective.pedestrian_detector_node:main'
        ],
    },
)