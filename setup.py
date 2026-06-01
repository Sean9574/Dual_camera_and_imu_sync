from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'igvc_camstuff'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'camera_node         = igvc_camstuff.camera_node:main',
            'camera_display_node = igvc_camstuff.camera_display_node:main',
            'imu_serial_node     = igvc_camstuff.imu_serial_node:main',
            'imu_monitor         = igvc_camstuff.imu_monitor:main',
            'sync_test           = igvc_camstuff.sync_test:main',
        ],
    },
)
