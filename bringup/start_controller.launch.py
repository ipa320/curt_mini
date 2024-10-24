from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["skid_steer_controller"],
        output="screen",
        remappings=[('odom', 'odometry/wheel')]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        remappings=[('odom', 'odometry/wheel')]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
