import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'diuniu_description' 
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'diuniu.urdf')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'diuniu.rviz')

    # 读取 URDF 文件内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 启动 robot_state_publisher 节点，发布 TF 坐标树
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 启动 RViz2 并自动加载配置好的 rviz 布局
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
