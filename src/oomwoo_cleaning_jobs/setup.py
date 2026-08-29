from setuptools import find_packages
from setuptools import setup

package_name = 'oomwoo_cleaning_jobs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_keepout.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oomwoo',
    maintainer_email='dev@example.com',
    description='ROS 2 nodes for cleaning intent: Nav2 keepout projection and job orchestration.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'constraint_mask_publisher = oomwoo_cleaning_jobs.constraint_mask_publisher:main',
        ],
    },
)
