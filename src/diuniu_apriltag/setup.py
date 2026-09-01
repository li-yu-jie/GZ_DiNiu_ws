from glob import glob
import os
from setuptools import setup

package_name = 'diuniu_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='y',
    maintainer_email='Liyujie@qq.com',
    description='DiuNiu 视觉感知包：Orbbec RGB 相机驱动 + 图像去畸变 + AprilTag 识别',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_info_relay = diuniu_apriltag.camera_info_relay:main',
            'tag_align = diuniu_apriltag.tag_align_node:main',
            'manual_align = diuniu_apriltag.manual_align:main',
        ],
    },
)
