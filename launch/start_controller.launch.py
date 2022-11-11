from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
