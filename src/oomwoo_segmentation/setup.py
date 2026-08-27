from setuptools import find_packages, setup

package_name = 'oomwoo_segmentation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oomwoo',
    maintainer_email='dev@example.com',
    description='Room segmentation based on the ROSE2 method: ROS 2 action server, client, and visualization tools.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oomwoo_segmentation_node = oomwoo_segmentation.node:main',
            'oomwoo-render-map = oomwoo_segmentation.render_map:main',
        ],
    },
)
