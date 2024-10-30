import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

def launch_robot(context, *args, **kwargs):
    # initialize arguments
    robot = LaunchConfiguration('robot')
    environment = LaunchConfiguration('environment')
    gear_ratio = LaunchConfiguration('gear_ratio')
    interface = LaunchConfiguration('interface')

    # start the base
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot.perform(context)), 'bringup', 'robot_base.launch.py')),
        launch_arguments={'robot' : robot,
                          'environment' : environment,
                          'gear_ratio' : gear_ratio,
                          'interface' : interface
                          }.items()
    )

    # start the navigation
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot.perform(context)), 'bringup', 'navigation_only.launch.py')),
        launch_arguments={'robot' : robot,
                          'environment' : environment
                          }.items()
    )

    return [
        base,
        nav
    ]

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

    # environment argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'environment',
            default_value='empty',
            description='Setup the environment map that will be loaded',
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_robot)])
