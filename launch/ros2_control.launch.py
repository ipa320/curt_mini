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
    robot = LaunchConfiguration('robot')
    gear_ratio = LaunchConfiguration('gear_ratio')
    interface = LaunchConfiguration('interface')

    robot_dir = get_package_share_directory(robot.perform(context))
    
    ros2_control_yaml_path = os.path.join(robot_dir, 'config', 'ros2_control.yaml')

    urdf_path = os.path.join(robot_dir, 'models', robot.perform(context), 'robot.urdf.xacro')

    robot_description = Command(['xacro ', urdf_path, ' gear_ratio:=', gear_ratio, ' interface:=', interface])
    
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_control_yaml_path,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    md80_manager_node = Node(
        package='candle_ros2',
        executable='candle_ros2_node',
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    return [controller_manager_node,
            md80_manager_node]

def generate_launch_description():

    declared_arguments = []

    # robot argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot',
            default_value='curt_mini',
            description="Set the robot.",
            choices=['curt_mvp', 'curt_mini'],
        )
    )

    # set the interface for the ethercat communication
    declared_arguments.append(
        DeclareLaunchArgument(
            'interface',
            default_value='enp0s31f6',
            description='Ethercat interface to the motor controllers',
        )
    )

    # set the gear_ratio
    declared_arguments.append(
        DeclareLaunchArgument(
            'gear_ratio',
            default_value='1',
            description='Set the gear ratio of the motors',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_ros2_control)])
