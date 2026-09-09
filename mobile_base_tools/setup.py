from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mobile_base_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Safwan',
    maintainer_email='safwan@example.com',
    description='Runtime utilities for the mobile mecanum base.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mecanum_motion_test = mobile_base_tools.mecanum_motion_test:main',
        'odom_to_path = mobile_base_tools.odom_to_path:main',
        'timestamp_validator = mobile_base_tools.timestamp_validator:main',
        'tof_ray_markers = mobile_base_tools.tof_ray_markers:main',
        'tof_floor_classifier = mobile_base_tools.tof_floor_classifier:main',
        'tf_validator = mobile_base_tools.tf_validator:main',
    ]},
)
