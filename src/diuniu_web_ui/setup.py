import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diuniu_web_ui'


def web_data_files():
    """递归收集前端产物，保持目录结构安装到 share/<pkg>/web/。"""
    entries = []
    root = os.path.join(package_name, 'web')
    if not os.path.isdir(root):
        return entries
    for dirpath, _dirnames, filenames in os.walk(root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, package_name)
        entries.append((f'share/{package_name}/{rel}',
                        [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Vue 前端构建产物（npm run build 后由 deploy 脚本拷入 web/）
        *web_data_files(),
    ],
    install_requires=[
        'setuptools',
        # 后端实际 import 的第三方包（websockets 有包内 _vendor 回退，故不强制）
        'fastapi',
        'uvicorn',
        'pydantic',
        'pyjwt',
        'bcrypt',
        'pillow',
        'pyyaml',
        'psutil',
    ],
    zip_safe=True,
    maintainer='y',
    maintainer_email='y@diuniu.local',
    description='地牛叉车 Web 控制端（FastAPI + JWT/RBAC + rosbridge）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_server = diuniu_web_ui.main:main',
        ],
    },
)
