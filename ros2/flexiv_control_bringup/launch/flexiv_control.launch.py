"""Bring up the flexiv_control ROS 2 bridge.

Examples
--------
Simulated (no hardware)::

    ros2 launch flexiv_control_bringup flexiv_control.launch.py backend:=fake

Real Rizon::

    ros2 launch flexiv_control_bringup flexiv_control.launch.py \\
        backend:=flexiv_rdk robot_serial:=Rizon4s-XXXXXX \\
        safety_profile:=tabletop_safe

Community project, NOT affiliated with Flexiv Robotics.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("backend", default_value="fake",
                              description="flexiv_control backend: fake | flexiv_rdk"),
        DeclareLaunchArgument("robot_serial", default_value="",
                              description="Rizon serial (required for flexiv_rdk)"),
        DeclareLaunchArgument("control_hz", default_value="100.0"),
        DeclareLaunchArgument("safety_profile", default_value="tabletop_safe"),
        DeclareLaunchArgument("state_rate", default_value="50.0"),
    ]

    node = Node(
        package="flexiv_control_bringup",
        executable="control_node",
        name="flexiv_control_node",
        output="screen",
        # Land JointState on the global /joint_states so robot_state_publisher /
        # RViz / joint_state consumers pick it up (the node publishes it on the
        # node-private ~/joint_states).
        remappings=[("~/joint_states", "/joint_states")],
        parameters=[{
            "backend": LaunchConfiguration("backend"),
            "robot_serial": LaunchConfiguration("robot_serial"),
            "control_hz": LaunchConfiguration("control_hz"),
            "safety_profile": LaunchConfiguration("safety_profile"),
            "state_rate": LaunchConfiguration("state_rate"),
        }],
    )

    return LaunchDescription(args + [node])
