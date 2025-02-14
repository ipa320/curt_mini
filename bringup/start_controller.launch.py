from launch import LaunchDescription
import launch.substitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
import typing


class ResultingParameterFilePath(launch.substitution.Substitution):
    """
    Substitution which resolves to the path of the temporary file created by ParameterFile
    """

    def __init__(self, parameter_file: ParameterFile) -> None:
        super().__init__()
        self.__parameter_file = parameter_file

    def perform(self, context: launch.launch_context.LaunchContext) -> typing.Text:
        return self.__parameter_file.evaluate(context).absolute().as_posix()


def generate_launch_description():

    param_file = ResultingParameterFilePath(
        ParameterFile(
            PathJoinSubstitution(
                [FindPackageShare("curt_mini"), "config", "ros2_control.yaml"]
            ),
            allow_substs=True,
        )
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
