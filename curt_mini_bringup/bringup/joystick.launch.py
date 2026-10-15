# Compatibility wrapper for existing curt_mini_bringup joystick launch commands.
# The implementation and configuration live in curt_mini_teleop.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("curt_mini_teleop"), "launch", "joystick.launch.py"
            ])
        ))
    ])
