from setuptools import find_packages, setup

package_name = 'tf_odom_link_base'

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
    maintainer='tanmay',
    maintainer_email='tanmaybains.2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tf_odom = tf_odom_link_base.tf_odom:main',
            'tf_map = tf_odom_link_base.tf_map:main',
        ],
    },
)
