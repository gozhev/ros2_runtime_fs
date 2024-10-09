import glob
import os

from setuptools import setup

package_name = 'ros2_runtime_fs'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch',
            glob.glob(os.path.join('launch', '*.launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mikhail Gozhev',
    maintainer_email='m@gozhev.org',
    description='A FUSE filesystem that exposes a ROS2 network of nodes as a hierarchy of directories and symlinks',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_runtime_fs = ros2_runtime_fs.ros2_runtime_fs:main',
        ],
    },
)
