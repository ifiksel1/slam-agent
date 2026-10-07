#!/usr/bin/env python3
"""
Dragunfly Gazebo Harmonic SITL bringup.

Starts: gz sim world + spawned Dragunfly + ros_gz bridge + robot_state_publisher.
Does NOT start: ArduPilot SITL, FAST-LIO2, MAVROS. Those are launched separately
so the sim can be restarted without tearing down the SLAM stack.

Typical session (4 terminals):
  1) ros2 launch dragunfly_description sim_launch.py
  2) sim_vehicle.py -v ArduCopter -f JSON --console --map
  3) ros2 launch <slam_pkg> slam_launch.py          # unchanged from hardware
  4) ros2 launch <mavros_pkg> mavros_bridge_launch.py

Requires GZ_VERSION=harmonic and the ArduPilot/ardupilot_gazebo plugin on
GZ_SIM_SYSTEM_PLUGIN_PATH.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dragunfly_description")

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_z = LaunchConfiguration("spawn_z")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "dragunfly.urdf.xacro"])
    bridge_cfg = PathJoinSubstitution([pkg, "config", "gz_bridge.yaml"])

    # sim:=true is the whole point of this launch file. The same xacro with
    # sim:=false is what runs on the aircraft.
    robot_description = Command(["xacro ", xacro_file, " sim:=true"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="empty.sdf",
            description="Gazebo world. Use a feature-rich indoor world to exercise "
                        "FAST-LIO2 properly; empty.sdf gives the LiDAR nothing to "
                        "match against and SLAM will legitimately diverge."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "spawn_z", default_value="0.15",
            description="Spawn height. Must clear the inverted LiDAR (6 cm below "
                        "base_link) or the sensor starts inside the ground plane."),

        # Gazebo Harmonic. -r starts unpaused; lock_step in ArduPilotPlugin keeps
        # physics and the autopilot in sync regardless.
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-v", "3", world],
            output="screen",
        ),

        # robot_state_publisher owns ALL static TFs, including os_lidar/os_imu.
        # The Ouster driver must stay at pub_static_tf: false on hardware so this
        # remains the single source of truth.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        # Rotor joints are 'continuous' in sim; publish their state so RViz does
        # not warn about missing joint values.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "/robot_description",
                "-name", "dragunfly",
                "-x", "0", "-y", "0", "-z", spawn_z,
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[{
                "config_file": bridge_cfg,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
