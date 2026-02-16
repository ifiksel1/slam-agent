#!/usr/bin/env python3
"""
MAVROS Bridge Launch File

Launches:
1. MAVROS via official apm.launch (XML) with ArduPilot connection
2. Vision bridge node for SLAM odometry forwarding
3. Static TF: map → odom (bridges MAVROS frames into SLAM TF tree)
4. Stream rate request (enables MAVLink data streams from ArduPilot)

Configuration:
- FCU: /dev/ttyACM0 at 921600 baud (ArduPilot Cube Orange)
- Vision bridge: Forwards /Odometry to /mavros/vision_pose/pose
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mavros_dir = get_package_share_directory('mavros')

    return LaunchDescription([
        # Static TF: map → odom (identity)
        # MAVROS creates odom → odom_ned, but odom is disconnected from
        # FAST-LIO's tree (map → camera_init → base_link).
        # This bridges them so Foxglove/TF2 can resolve all frame paths.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_odom',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
            ],
            output='screen',
        ),

        # MAVROS via official APM launch file
        # This correctly loads plugins, config, and namespacing
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': '/dev/ttyACM0:921600',
                'gcs_url': '',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items(),
        ),

        # Vision bridge: /Odometry → /mavros/vision_pose/pose
        ExecuteProcess(
            cmd=['bash', '-c',
                 'source /opt/ros/humble/setup.bash && python3 /ws/vision_bridge.py'],
            name='vision_bridge',
            output='screen',
        ),

        # Request all MAVLink streams at 10 Hz after MAVROS connects
        # stream_id=0 means ALL streams (local_position, imu, battery, etc.)
        # Delay 10s to allow MAVROS to establish connection first
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                         'source /opt/ros/humble/setup.bash && '
                         'ros2 service call /mavros/set_stream_rate '
                         'mavros_msgs/srv/StreamRate '
                         '"{stream_id: 0, message_rate: 10, on_off: true}" && '
                         'ros2 service call /mavros/set_stream_rate '
                         'mavros_msgs/srv/StreamRate '
                         '"{stream_id: 6, message_rate: 50, on_off: true}"'],
                    name='set_stream_rate',
                    output='screen',
                ),
            ],
        ),
    ])
