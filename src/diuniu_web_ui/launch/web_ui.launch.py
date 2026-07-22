# =============================================================================
# web_ui.launch.py — Web 控制端一键启动：rosbridge(:9090) + FastAPI(:8000)
#
# 用法：
#   ros2 launch diuniu_web_ui web_ui.launch.py
# 然后浏览器访问 http://<工控机IP>:8000
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # rosbridge WebSocket 服务（前端 roslibjs 直连 ROS2 话题/服务/Action）
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': '9090',
                          'address': '0.0.0.0',
                          'retry_startup_delay': '5.0'}.items()
    )

    # FastAPI 后端（静态页托管 + REST API）
    web_server_node = Node(
        package='diuniu_web_ui',
        executable='web_server',
        name='diuniu_web_server',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        web_server_node
    ])
