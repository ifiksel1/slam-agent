#!/usr/bin/env python3
"""
FAST-LIO-GPU SLAM System Launch File (ROS 2 Humble)
Jetson Orin NX + Ouster OS1-64 + ArduPilot Cube Orange
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.getenv('SLAM_CONFIG_DIR', '/opt/slam_ws/config')
    fast_lio_config = os.path.join(config_dir, 'fast_lio_gpu.yaml')
    ouster_config = os.path.join(config_dir, 'ouster_driver.yaml')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros', default_value='false')
    enable_ouster_arg = DeclareLaunchArgument(
        'enable_ouster', default_value='true')
    enable_foxglove_arg = DeclareLaunchArgument(
        'enable_foxglove', default_value='true')

    # ── Robot State Publisher (URDF → TF) ──
    urdf_content = ''
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
    else:
        urdf_content = (
            '<?xml version="1.0"?>'
            '<robot name="slam_robot">'
            '<link name="base_link"/>'
            '<link name="os_lidar"/>'
            '<joint name="lidar_joint" type="fixed">'
            '<parent link="base_link"/>'
            '<child link="os_lidar"/>'
            '<origin xyz="0 0 0.1" rpy="0 0 0"/>'
            '</joint>'
            '</robot>'
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ── Ouster OS1-64 Driver (LifecycleNode via included launch) ──
    # The ouster_ros driver.launch.py handles configure→activate lifecycle
    ouster_ros_pkg = get_package_share_directory('ouster_ros')
    ouster_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ouster_ros_pkg) / 'launch' / 'driver.launch.py')
        ),
        launch_arguments={
            'params_file': ouster_config,
            'ouster_ns': 'ouster',
            'os_driver_name': 'os_driver',
            'viz': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_ouster')),
    )

    # ── FAST-LIO-GPU SLAM ──
    fast_lio_gpu = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ── Static TF: map → camera_init ──
    # FAST-LIO publishes camera_init → body (base_link).
    # We need map → camera_init to complete the chain.
    tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'camera_init',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
        output='screen',
    )

    # ── MAVROS (optional, disabled by default) ──
    mavros_config = os.path.join(config_dir, 'mavros_config.yaml')
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mavros')),
        parameters=[mavros_config, {
            'fcu_url': '/dev/ttyUSB0:921600',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'system_id': 1,
            'component_id': 191,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ── Foxglove Bridge (WebSocket visualization) ──
    # Enables Foxglove Studio to connect at ws://<jetson-ip>:8765
    foxglove_pkg = get_package_share_directory('foxglove_bridge')
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            str(Path(foxglove_pkg) / 'launch' / 'foxglove_bridge_launch.xml')
        ),
        launch_arguments={
            'port': '8765',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_mavros_arg,
        enable_ouster_arg,
        enable_foxglove_arg,
        LogInfo(msg='Starting FAST-LIO-GPU SLAM system...'),
        robot_state_publisher,
        ouster_driver_launch,
        fast_lio_gpu,
        tf_map_to_camera_init,
        mavros_node,
        foxglove_bridge_launch,
    ])
