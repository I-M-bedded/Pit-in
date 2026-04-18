from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_handeye'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # ros2_aruco 브릿지 방식 사용
            'aruco_to_tf_bridge = my_robot_handeye.aruco_to_tf_bridge:main',
        ],
    },
)