# =============================================================================
# n10_mapping.launch.py — N10 雷达 slam_toolbox 建图一键启动
#
# 启动组件：
#   1. lslidar_driver — N10 驱动 → /scan (n10_laser_link)
#   2. robot_state_publisher — URDF 静态 TF（含 base_link→n10_laser_link）
#   3. diuniu_base — 底盘（/wheel_odom + /imu/data 常发；odom TF/topic 关闭）
#   4. EKF (robot_localization) — 融合轮程 vx/wz + BNO085 航向 → odom→base_link TF
#      ★ N10 栈无 FAST-LIO，EKF 是唯一的 odom→base_link 源，必须启动
#   5. laserscan_filter — /scan → /scan_filtered（车身自遮挡屏蔽盒）
#   6. slam_toolbox (online_async) — 发布 map→odom TF 与 /map
#
# 建图流程：
#   1. 本 launch 启动后，另开终端启动手柄遥控：
#        ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
#   2. RViz 看 /map 边走边建（rviz:=true 可让本 launch 代开）
#   3. 跑完全场后保存地图（另开终端）：
#        ros2 run nav2_map_server map_saver_cli -f ~/GZ_DiNiu_ws/src/diuniu_n10_nav/maps/map
#
# 用法：
#   ros2 launch diuniu_n10_nav n10_mapping.launch.py
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_n10 = get_package_share_directory('diuniu_n10_nav')
    pkg_diuniu_base = get_package_share_directory('diuniu_base')
    pkg_description = get_package_share_directory('diuniu_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    urdf_file = os.path.join(pkg_description, 'urdf', 'diuniu.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='是否使用仿真时间（实车必须为 false）')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/n10_lidar', description='N10 串口设备')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否打开 RViz 实时查看建图（默认开，无显示环境时传 rviz:=false）')

    # 1. N10 驱动
    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg_n10, 'config', 'lsx10.yaml'),
                    {'serial_port_': serial_port}],
    )

    # 2. URDF 静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # 3. 底盘驱动（关闭自带 odom TF/topic，由 EKF 统一发布；/wheel_odom 常发）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diuniu_base, 'launch', 'diuniu_base.launch.py')
        ),
        launch_arguments={
            'pub_odom_tf': 'false',
            'pub_odom_topic': 'false'
        }.items()
    )

    # 4. EKF：轮程 vx/wz + BNO085 绝对航向 → odom→base_link TF（N10 栈必选）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_n10, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 5. 雷达自遮挡过滤器：/scan → /scan_filtered
    #    参数与 diuniu_nav/launch/diuniu_nav.launch.py 保持一致（改动请两边同步）：
    #    x_min=-0.35 车身后界+5cm（货叉在车身范围内、车尾无伸出，勿退回 -1.65 幻影尾巴）；
    #    x_max=1.65 / y=±0.36 车身轮廓+1cm（±0.40 会误删贴侧行人）
    #    ★ laser_x_offset=1.295：过滤器不走 TF，直接在扫描坐标系算 x_base=x_laser+offset
    #      （Mid360 链 /scan 已是 base_link 系故 offset=0；N10 的 /scan 是 n10_laser_link
    #       系，必须给雷达在 base_link 下的 x 偏移(原雷达位前方 8cm)，否则屏蔽盒错位滤不到车身）
    #    ⚠️ 过滤器不支持旋转补偿：N10 安装 yaw 必须物理对准车头（URDF n10_laser_joint yaw=0）
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.35,
            'x_max': 1.65,
            'y_min': -0.36,
            'y_max': 0.36,
            'laser_x_offset': 1.295,   # ★ N10 在 base_link 前方 1.295m（原雷达位前 8cm，与 URDF 同步）
            'laser_y_offset': 0.0,
            'arc_filter_enabled': False
        }],
        output='screen'
    )

    # 6. slam_toolbox online_async 建图
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_n10, 'config', 'slam_toolbox.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # 可选 RViz（默认开）：固定系 map，显示 /map + /scan_filtered + TF
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_n10, 'rviz', 'n10_mapping.rviz')],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_serial_port,
        declare_rviz,
        driver_node,
        robot_state_publisher_node,
        base_launch,
        ekf_node,
        laserscan_filter,
        slam_launch,
        rviz_node,
    ])
