import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def launch_robot(context, *args, **kwargs):
    # initialize arguments
    robot = LaunchConfiguration('robot')
    environment = LaunchConfiguration('environment')
    gear_ratio = LaunchConfiguration('gear_ratio')
    interface = LaunchConfiguration('interface')

    twist_pkg_name = robot.perform(context)
    twist_pkg_path = get_package_share_directory(twist_pkg_name)
    twist_mux_path = os.path.join(twist_pkg_path, 'config/twist_mux.yaml')

    # start the hardware interface
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ipa_ros2_control'), 'launch', 'ros2_control.launch.py')),
        launch_arguments={'robot' : robot,
                          'environment' : environment,
                          'gear_ratio' : gear_ratio,
                          'interface' : interface}.items()
    )

    # start the controllers
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot.perform(context)), 'launch', 'start_controller.launch.py')),
    )

    # start joystick
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot.perform(context)), 'bringup', 'joystick.launch.py')),
        launch_arguments={'robot' : LaunchConfiguration('robot')}.items()
    )

    zero_twist = Node(
            package='ipa_twist_mux',
            executable='zero_twist_publisher',
            name='zero_twist_publisher',
            output='screen',
            remappings=[('/twist', '/zero_twist/cmd_vel')],
            parameters=[{'use_sim_time': False}]
            )

    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_path,
                        {'use_sim_time': False}],
            #remappings=[('/cmd_vel_out', '/skid_steer_controller/cmd_vel_unstamped')]
            remappings=[('/cmd_vel_out', '/cmd_vel_unsafe')]
            #/skid_steer_controller/cmd_vel_unstamped'
    )
    pointcloud_safety = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ipa_outdoor_safety'), 'launch', 'pcl_collision_avoidance.launch.py')),
            launch_arguments={'use_sim_time' : 'False',
                            'robot' : robot,
                            'pointcloud_topic' : '/lidar_points',
                            'cmd_vel_in': '/cmd_vel_mux',
                            'cmd_vel_out': '/skid_steer_controller/cmd_vel_unstamped'}.items()
    )


    return [
        hardware_interface,
        controller,
        joystick,
        # pointcloud_safety,
        zero_twist,
        twist_mux
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
            choices=['empty', 'acker', 'plant_acker', 'uneven_terrain', 'kogrob_acker'],
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
