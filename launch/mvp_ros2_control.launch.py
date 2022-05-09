import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.conditions.launch_configuration_equals import LaunchConfigurationEquals
from launch.conditions import IfCondition
import xacro

def generate_launch_description():
    sim_dir = get_package_share_directory('ipa_outdoor_simulation')
    driver_dir = get_package_share_directory('ipa_outdoor_drivers')
    
    ros2_control_yaml_path = os.path.join(driver_dir, 'config', 'ros2_control.yaml')

    robot_description_path = os.path.join(sim_dir, 'urdf', 'mvp.urdf.xacro')

    declare_gear_ratio_cmd = DeclareLaunchArgument('gear_ratio', default_value='32')
    declare_interface_cmd = DeclareLaunchArgument(
        'interface',
        default_value='enp0s31f6',
        description='Ethercat interface to the motor controllers')

    gear_ratio = LaunchConfiguration('gear_ratio')
    interface = LaunchConfiguration('interface')

    robot_description = Command(['xacro ', robot_description_path, ' gear_ratio:=', gear_ratio, ' interface:=', interface])
    #robot_description = xacro.process_file(robot_description_path).toxml()


    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_yaml_path
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_drive_controller"],
    #     output="screen",
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # )

    return LaunchDescription([
        #joint_state_broadcaster_spawner,
        #robot_controller_spawner,
        declare_gear_ratio_cmd,
        declare_interface_cmd,
        controller_manager_node
    ])
