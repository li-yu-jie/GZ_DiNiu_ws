import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'diuniu_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'),
            glob('diuniu_task_manager/templates/*')),
    ],
    package_data={
        package_name: ['templates/*'],
    },
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fms_node = diuniu_task_manager.fms_node:main',
            'web_server = diuniu_task_manager.web_server:main',
        ],
    },
)
