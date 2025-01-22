import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import UnlessCondition

sim_configuration = LaunchConfiguration("simulation")

def launch_robot(context, *args, **kwargs):
    # initialize arguments
    #robot = "curt_mini"
    robot = LaunchConfiguration('robot')
    gear_ratio = LaunchConfiguration('gear_ratio')
    interface = LaunchConfiguration('interface')
    robot_dir = FindPackageShare(robot)
    ouster_path = PathJoinSubstitution([robot_dir, "config", "ouster.yaml"])
    imu_xsens_path = PathJoinSubstitution([robot_dir, "config", "xsens.yaml"])


    twist_pkg_name = robot.perform(context)
    twist_pkg_path = get_package_share_directory(twist_pkg_name)
    twist_mux_path = os.path.join(twist_pkg_path, 'config/twist_mux.yaml')

    # start the state publisher
    urdf_path = PathJoinSubstitution([robot_dir, "models", robot, "robot.urdf.xacro"])
    robot_description = Command(["xacro ", urdf_path, 
                                 " interface:=", interface,
                                 " simulation:=", sim_configuration,
                                ])
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    # start the hardware interface
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ipa_ros2_control'), 'launch', 'ros2_control.launch.py'])),
        launch_arguments={'robot' : robot,
                          'gear_ratio' : gear_ratio,
                          'interface' : interface}.items()
    )

    # start the controllers
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot.perform(context)), 'bringup', 'start_controller.launch.py')),
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
            remappings=[('/cmd_vel_out', '/skid_steer_controller/cmd_vel_unstamped')]
            #remappings=[('/cmd_vel_out', '/cmd_vel_unsafe')]
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

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ouster_ros"), "launch", "driver.launch.py"]
            )
        ),
        launch_arguments={"viz": "False", "params_file": ouster_path}.items(),
    )

    imu_xsens = Node(
        package="ipa_xsens_mti_driver",
        name="xsens_mti_node",
        executable="xsens_mti_node",
        parameters=[imu_xsens_path],
    )

    imu_lpresearch = Node(
        package="openzen_driver",
        namespace="imu",
        executable="openzen_node",
        parameters=[
            {"sensor_interface": "LinuxDevice"},
            {"sensor_name": "devicefile:/dev/ttyLPMSCA3D00510053"},
        ],
    )

    realsense = Node(
        package="realsense2_camera",
        name="realsense2_camera_node",
        executable="realsense2_camera_node",
    )

    zenoh_bridge = ExecuteProcess(
        cmd=[
            FindExecutable(name="zenoh-bridge-ros2dds"),
            "-c",
            PathJoinSubstitution([robot_dir, "config", "zenoh_cfg.json5"]),
        ],
    )


    return [
        zenoh_bridge,
        TimerAction(
            period=5.0,
            actions=[
                state_publisher,
                controller,
                joystick,
                twist_mux,
                zero_twist,
                # Skip hardware interfaces when running in simulation
                GroupAction(
                    [
                        hardware_interface,
                        # imu_xsens,
                        imu_lpresearch,
                        lidar,
                        realsense,
                    ],
                    condition=UnlessCondition(sim_configuration),
                ),
            ]
        )
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation",
            default_value="False",
            choices=["True", "False"],
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_robot)])
