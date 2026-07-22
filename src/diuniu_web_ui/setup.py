from setuptools import setup
from glob import glob

package_name = 'diuniu_web_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Web 前端静态文件（含子目录 js/css/lib）
        ('share/' + package_name + '/web', glob('web/*.*')),
        ('share/' + package_name + '/web/css', glob('web/css/*')),
        ('share/' + package_name + '/web/js', glob('web/js/*.js')),
        ('share/' + package_name + '/web/js/lib', glob('web/js/lib/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LYJ',
    maintainer_email='lyj@example.com',
    description='地牛叉车 Web 控制端',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_server = diuniu_web_ui.main:main',
        ],
    },
)
