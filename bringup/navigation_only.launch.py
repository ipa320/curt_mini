import os

from launch.action import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    NotEqualsSubstitution,
)
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_nav() -> list[Action]:
    # initialize arguments
    environment = LaunchConfiguration('environment')
    robot = "curt_mini"

    # start the Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ipa_outdoor_navigation_bringup'), 'bringup', 'outdoor_navigation.launch.py')),
        launch_arguments={'robot' : robot,
                          'environment' : environment,
                          'launch_nav' : "True",
                          'pointcloud_topic' : "/rslidar_points",
                          }.items()
    )

    return [
        navigation
    ]


def generate_launch_description():
    declared_arguments = []

    # robot argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot',
            default_value='curt_mini',
            description="Set the robot.",
            choices=['curt_diff', 'curt_mini'],
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("curt_mini"), "config", "nav_setup.rviz"]
            ),
        ],        
        condition=IfCondition(
            NotEqualsSubstitution(EnvironmentVariable("DISPLAY", default_value=""), "")
        ),

    )

    return LaunchDescription(declared_arguments + launch_nav() + [rviz_node])
