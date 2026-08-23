from setuptools import find_packages
from setuptools import setup

package_name = 'oomwoo_cleaning_jobs_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oomwoo',
    maintainer_email='dev@example.com',
    description='Region Set editing, constraints, validation, and persistence.',
    license='Apache 2.0',
    tests_require=['pytest'],
)
