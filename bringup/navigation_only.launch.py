import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def launch_nav(context, *args, **kwargs):
    # initialize arguments
    robot = LaunchConfiguration('robot')
    environment = LaunchConfiguration('environment')

    # start the Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ipa_outdoor_navigation_bringup'), 'bringup', 'outdoor_navigation.launch.py')),
        launch_arguments={'robot' : robot,
                          'environment' : environment,
                          'launch_nav' : "True",
                          'launch_lidar_loc' : "True",
                          'pointcloud_topic' : "/ouster/points",
                          'use_sim_time' : 'False'}.items()
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_nav)])
