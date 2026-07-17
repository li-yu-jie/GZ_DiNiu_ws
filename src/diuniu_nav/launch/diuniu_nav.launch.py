import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    pkg_nav = get_package_share_directory('diuniu_nav')
    pkg_nav_bringup = get_package_share_directory('nav2_bringup')
    pkg_description = get_package_share_directory('diuniu_description')

    # Load URDF for robot_state_publisher
    urdf_file = os.path.join(pkg_description, 'urdf', 'diuniu.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Defaults
    default_map = os.path.join(pkg_nav, 'maps', 'map.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Launch Configurations
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')

    # Declare Launch Arguments
    declare_map = DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params, description='Full path to nav2 params file')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time')
    declare_use_amcl = DeclareLaunchArgument('use_amcl', default_value='true', description='Whether to launch AMCL')

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # Group 1: When use_amcl is True (Standard bringup with AMCL)
    bringup_with_amcl = GroupAction(
        condition=IfCondition(use_amcl),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # Group 2: When use_amcl is False (Localization direct via FAST-LIO, map_server + navigation_launch.py, NO AMCL)
    bringup_without_amcl = GroupAction(
        condition=UnlessCondition(use_amcl),
        actions=[
            # 1. Map Server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[params_file, {'yaml_filename': map_yaml}]
            ),
            # 2. Lifecycle Manager for map_server
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server']}]
            ),
            # 3. Navigation Server (planner, controller, behavior, bt_navigator)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'params_file': params_file
                }.items()
            ),
            # 4. Static TF map -> odom
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            )
        ]
    )

    # 3D PointCloud to 2D LaserScan Node (Always active to feed the costmaps and AMCL)
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.15,
            'max_height': 1.2,
            'angle_min': -3.1415926,
            'angle_max': 3.1415926,
            'angle_increment': 0.0087,  # 0.5 degrees
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
            'concurrency_level': 0,
            'queue_size': 2
        }],
        output='screen'
    )

    # LaserScan filter node to filter out self-occlusions (forks, body structures)
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.25,
            'x_max': 1.60,
            'y_min': -0.35,
            'y_max': 0.35,
            'laser_x_offset': 0.0,
            'laser_y_offset': 0.0
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_use_amcl,
        robot_state_publisher_node,
        bringup_with_amcl,
        bringup_without_amcl,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
