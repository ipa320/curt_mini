from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def host_is_up(host: str) -> bool:
    # -c: number of pings, -W: timeout in seconds
    return os.system(f"ping -c 1 -W 0.2 {host} >/dev/null") == 0


# Try determining the robot IP by trying to reach each candidate in order.
def find_default_robot_ip() -> str:
    tailscale_host = "nuc-curt-mini"
    tailscale_ip = "100.64.0.10"
    nighthawk_ip = "192.168.130.46"
    hotspot_ip = "10.42.1.1"

    # Hosts in priority order
    possible_hosts = [nighthawk_ip, hotspot_ip, tailscale_host, tailscale_ip]
    for host in possible_hosts:
        if host_is_up(host):
            return host
    return tailscale_host


def launch_robot():
    robot_ip = LaunchConfiguration("robot_ip")

    # State publisher: Start to workaround https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/issues/333
    # which affects /tf_static from robot state publisher, such as base_link -> lidar_link TF
    robot = "curt_mini"
    robot_dir = FindPackageShare(robot)
    urdf_path = PathJoinSubstitution([robot_dir, "models", robot, "robot.urdf.xacro"])
    robot_description = Command(
        [
            "xacro ",
            urdf_path,
            " interface:=/dev/null",
            " simulation:=False",
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    zenoh_bridge = ExecuteProcess(
        cmd=[
            FindExecutable(name="zenoh-bridge-ros2dds"),
            "-e",
            ["tcp/", robot_ip, ":7447"],
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(robot), "config", "nav_setup.rviz"]
            ),
        ],
    )

    return [
        zenoh_bridge,
        rviz_node,
        robot_state_publisher,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", default_value=find_default_robot_ip())
    )

    return LaunchDescription(declared_arguments + launch_robot())
