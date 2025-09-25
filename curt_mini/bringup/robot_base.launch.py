import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
)
from launch.conditions import UnlessCondition


def find_serial_device_by_prefix(prefix: str, default: str) -> str:
    entries = [f for f in os.listdir("/dev") if f.startswith(prefix)]
    if len(entries) == 1:
        return entries[0]
    else:
        return default


def launch_robot():
    # initialize arguments
    robot = "curt_mini"
    sim_configuration = LaunchConfiguration("simulation")
    robot_dir = FindPackageShare(robot)

    twist_mux_path = PathJoinSubstitution([robot_dir, "config", "twist_mux.yaml"])

    # start the state publisher
    urdf_path = PathJoinSubstitution([robot_dir, "models", robot, "robot.urdf.xacro"])
    robot_description = Command(
        [
            "xacro ",
            urdf_path,
            " simulation:=",
            sim_configuration,
        ]
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    # start the hardware interface
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ipa_ros2_control"),
                    "launch",
                    "ros2_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot": robot,
        }.items(),
    )

    # start the controllers
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    robot_dir,
                    "bringup",
                    "start_controller.launch.py",
                ]
            )
        ),
    )

    # start joystick
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    robot_dir,
                    "bringup",
                    "joystick.launch.py",
                ]
            )
        ),
        launch_arguments={"robot": robot}.items(),
    )

    zero_twist = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--rate",
            "20",
            "--print",
            "0",
            "/zero_twist/cmd_vel",
            "geometry_msgs/msg/TwistStamped",
            "{header: {stamp: now, frame_id: 'base_link'}}",
        ],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_path, {"use_sim_time": False}],
        remappings=[("/cmd_vel_out", "/skid_steer_controller/cmd_vel_unstamped")],
    )

    imu_lpresearch = Node(
        package="openzen_driver",
        namespace="imu",
        executable="openzen_node",
        parameters=[
            {"sensor_interface": "LinuxDevice"},
            {
                "sensor_name": "devicefile:/dev/"
                + find_serial_device_by_prefix("ttyLPMS", "ttyLPMSCA3D00510053")
            },
            {"frame_id": "imu_link"},
        ],
    )

    return [
        state_publisher,
        controller,
        joystick,
        twist_mux,
        zero_twist,
        # Skip hardware interfaces when running in simulation
        GroupAction(
            [
                hardware_interface,
                imu_lpresearch,
            ],
            condition=UnlessCondition(sim_configuration),
        ),
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation",
            default_value="False",
            choices=["True", "False"],
        )
    )

    return LaunchDescription(declared_arguments + launch_robot())
