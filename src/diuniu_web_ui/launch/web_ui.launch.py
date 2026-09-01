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
    # rosbridge WebSocket 服务（前端 roslibjs 经 FastAPI /ws/rosbridge 鉴权代理接入）
    # address=127.0.0.1：只对本地代理开放，外部唯一直连 ROS 的入口被关掉，
    # 否则 REST 层的 RBAC 会被直连 9090 端口旁路。
    # send_action_goals_in_new_thread=true：每个 action 目标在独立线程处理，
    # 否则一次导航会阻塞整条 WS 连接（地图/TF/摇杆全部卡死直到导航结束）
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': '9090',
                          'address': '127.0.0.1',
                          'retry_startup_delay': '5.0',
                          'send_action_goals_in_new_thread': 'true'}.items()
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
