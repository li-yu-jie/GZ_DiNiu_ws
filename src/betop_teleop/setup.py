from setuptools import setup
from glob import glob
import os

package_name = 'betop_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='北通鲲鹏20手柄遥控',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diuniu_joy_publisher = betop_teleop.diuniu_joy_publisher:main',
            'diuniu_teleop_serial = betop_teleop.diuniu_teleop_serial:main',
            'diuniu_teleop_cmd_vel = betop_teleop.diuniu_teleop_cmd_vel:main',
        ],
    },
)
