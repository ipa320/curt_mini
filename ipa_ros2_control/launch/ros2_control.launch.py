from email.policy import default
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_ros2_control(context, *args, **kwargs):
    # initialize arguments
    robot = "curt_mini"

    robot_dir = get_package_share_directory(robot)

    ros2_control_yaml_path = os.path.join(robot_dir, "config", "ros2_control.yaml")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_control_yaml_path,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    md80_manager_node = Node(
        package="candle_ros2",
        executable="candle_ros2_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=["USB", "1M"],
    )
    return [controller_manager_node, md80_manager_node]


def generate_launch_description():

    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_ros2_control)]
    )
