from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    param_file = PathJoinSubstitution(
        [FindPackageShare("curt_mini"), "config", "ros2_control.yaml"]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", param_file, "skid_steer_controller"],
        output="screen",
        remappings=[('odom', 'odometry/wheel')]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", param_file, "joint_state_broadcaster"],
        output="screen",
        remappings=[('odom', 'odometry/wheel')]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
