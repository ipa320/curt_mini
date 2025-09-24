import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_joystick(context, *args, **kwargs):
    # initialize arguments
    robot = "curt_mini"

    filepath_config_joy = os.path.join(
        get_package_share_directory(robot), "config", "joystick.yaml"
    )
    print(filepath_config_joy)
    node_joy = Node(
        namespace="joy_teleop",
        package="joy_linux",
        executable="joy_linux_node",
        output="screen",
        name="joy_node",
        parameters=[filepath_config_joy],
    )

    node_teleop_twist_joy = Node(
        namespace="joy_teleop",
        package="teleop_twist_joy",
        executable="teleop_node",
        output="screen",
        name="teleop_twist_joy_node",
        # remappings=[
        #     ('/joy_teleop/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        # ],
        parameters=[filepath_config_joy],
    )

    return [node_joy, node_teleop_twist_joy]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_joystick)]
    )
