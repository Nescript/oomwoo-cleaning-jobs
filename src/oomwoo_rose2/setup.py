from glob import glob
from setuptools import find_packages, setup

package_name = 'oomwoo_rose2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'THIRD_PARTY.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='oomwoo',
    maintainer_email='dev@example.com',
    description='GPLv3 ROS 2 port of the ROSE + ROSE2 room-segmentation pipeline.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oomwoo-rose2-server = oomwoo_rose2.node:main',
        ],
    },
)
