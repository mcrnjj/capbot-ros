"""
robot.launch.py - bringup del robot REAL en la Jetson (ROS2 Eloquent).

OJO SINTAXIS ELOQUENT: la accion Node usa 'node_executable' y 'node_name'
(en Foxy+ se llaman 'executable' y 'name'). Si migras a Foxy, renombralos.

Trae:
  - robot_state_publisher (URDF real, sin lidar/gazebo)
  - csi_camera_node      (camara CSI -> /camera/image_raw + /camera/camera_info)
  - esp32_bridge_node    (puente a capbot-jetson-bridge: from_bridge -> /odom ;
                          /cmd_vel -> to_bridge. El servicio jetson-bridge,
                          en OTRO proceso, es quien tiene el puerto serie.)
  - aruco_localizer      (-> /aruco_pose)
  - ekf x2 (robot_localization): local (odom->base_link) y global (map->odom)
  - [si enable_nav:=true] map_server + NAV2 (planner/controller/recoveries/
    bt_navigator/waypoint_follower) + lifecycle managers

Uso:
  ros2 launch test_bot robot.launch.py
  ros2 launch test_bot robot.launch.py enable_nav:=false        # solo localizacion
  ros2 launch test_bot robot.launch.py map_name:=large
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

PKG = get_package_share_directory('test_bot')


def generate_launch_description():
    enable_nav = LaunchConfiguration('enable_nav')
    map_name = LaunchConfiguration('map_name')
    map_file = LaunchConfiguration('map_file')
    markers_db = LaunchConfiguration('markers_db')

    nav2_params = os.path.join(PKG, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(PKG, 'config', 'ekf.yaml')
    camera_info = 'file://' + os.path.join(PKG, 'config', 'camera.yaml')

    xacro_file = os.path.join(PKG, 'description', 'robot_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ----- nodos base (localizacion) -----
    rsp = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[xacro_file],
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    camera = Node(
        package='test_bot', node_executable='csi_camera_node',
        node_name='csi_camera_node', output='screen',
        parameters=[{
            'sensor_id': 0,
            'capture_width': 1280, 'capture_height': 720,
            'output_width': 640, 'output_height': 480,
            'framerate': 30, 'flip_method': 0,
            'frame_id': 'camera_link_optical',
            'camera_info_url': camera_info,
            'publish_rate': 30.0,
        }],
    )

    esp32 = Node(
        package='test_bot', node_executable='esp32_bridge_node',
        node_name='esp32_bridge_node', output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'publish_odom_tf': False,   # el EKF local publica odom->base_link
            'max_linear_speed': 0.3,    # m/s; ajustar a velocidad maxima real del robot
            'max_angular_speed': 2.0,   # rad/s; idem
            'cmd_vel_timeout': 0.5,
        }],
    )

    aruco = Node(
        package='test_bot', node_executable='aruco_localizer',
        node_name='aruco_localizer', output='screen',
        parameters=[{
            'use_sim_time': False,
            'markers_db': markers_db,
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'camera_frame': 'camera_link_optical',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'publish_tf': False,
            'max_distance': 2.0,
            'max_reproj_error_px': 3.0,
            'min_marker_area_px': 200.0,
            'filter_window': 1,
            'ambiguity_ratio_threshold': 1.5,
        }],
    )

    ekf_odom = Node(
        package='robot_localization', node_executable='ekf_node',
        node_name='ekf_filter_node_odom', output='screen',
        parameters=[ekf_params],
        remappings=[('/odometry/filtered', '/odometry/filtered_odom')],
    )
    ekf_map = Node(
        package='robot_localization', node_executable='ekf_node',
        node_name='ekf_filter_node_map', output='screen',
        parameters=[ekf_params],
        remappings=[('/odometry/filtered', '/odometry/filtered_map')],
    )

    # ----- NAV2 (condicional) -----
    map_server = Node(
        package='nav2_map_server', node_executable='map_server',
        node_name='map_server', output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': False}],
        condition=IfCondition(enable_nav),
    )
    planner = Node(
        package='nav2_planner', node_executable='planner_server',
        node_name='planner_server', output='screen', parameters=[nav2_params],
        condition=IfCondition(enable_nav),
    )
    controller = Node(
        package='nav2_controller', node_executable='controller_server',
        node_name='controller_server', output='screen', parameters=[nav2_params],
        condition=IfCondition(enable_nav),
    )
    recoveries = Node(
        package='nav2_recoveries', node_executable='recoveries_server',
        node_name='recoveries_server', output='screen', parameters=[nav2_params],
        condition=IfCondition(enable_nav),
    )
    bt_nav = Node(
        package='nav2_bt_navigator', node_executable='bt_navigator',
        node_name='bt_navigator', output='screen', parameters=[nav2_params],
        condition=IfCondition(enable_nav),
    )
    wp_follower = Node(
        package='nav2_waypoint_follower', node_executable='waypoint_follower',
        node_name='waypoint_follower', output='screen', parameters=[nav2_params],
        condition=IfCondition(enable_nav),
    )
    lifecycle_map = Node(
        package='nav2_lifecycle_manager', node_executable='lifecycle_manager',
        node_name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'],
                     'use_sim_time': False}],
        condition=IfCondition(enable_nav),
    )
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager', node_executable='lifecycle_manager',
        node_name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['planner_server', 'controller_server',
                                    'recoveries_server', 'bt_navigator',
                                    'waypoint_follower'],
                     'use_sim_time': False}],
        condition=IfCondition(enable_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_nav', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='small'),
        DeclareLaunchArgument(
            'map_file',
            default_value=[PKG, '/config/test_map_', map_name, '.yaml']),
        DeclareLaunchArgument(
            'markers_db',
            default_value=[PKG, '/config/markers_db_', map_name, '.yaml']),

        rsp, camera, esp32, aruco, ekf_odom, ekf_map,
        map_server, planner, controller, recoveries, bt_nav, wp_follower,
        lifecycle_map, lifecycle_nav,
    ])
