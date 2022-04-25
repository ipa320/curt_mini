import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions.launch_configuration_equals import LaunchConfigurationEquals
from launch.conditions import IfCondition
import xacro

def generate_launch_description():
    sim_dir = get_package_share_directory('ipa_outdoor_simulation')
    driver_dir = get_package_share_directory('ipa_outdoor_drivers')
    
    ros2_control_yaml_path = os.path.join(driver_dir, 'config', 'ros2_control.yaml')

    robot_description_path = os.path.join(sim_dir, 'urdf', 'mvp.urdf.xacro')
    robot_description = xacro.process_file(robot_description_path).toxml()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller", "-c", "/controller_manager"],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_yaml_path
        ],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf')
        ],
        output='screen'
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        controller_manager_node
    ])
