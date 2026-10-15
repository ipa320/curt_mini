# This is the jazzy-devel branch

# Curt Mini

Select a branch and the ROS version for your robot. You may need a ROS1 and a ROS2 workspace.

## General Notes

This repository is used for setting up and starting the CURTmini software stack.
It consists of the configurations and dependencies for the sensor equipment on the robot.
In the bringup folder you find the launchfiles for starting the base and the whole navigation.


## Setting up the jazzy workspace

```
mkdir -p <colcon_ws>/src
cd <colcon_ws>/src 
git clone https://github.com/ipa320/curt_mini.git
cd ..
vcs import --recursive src/ < src/curt_mini/curt_mini_bringup/curt_mini.repos
vcs import --recursive src/ < src/curt_mini/ipa_ros2_control/ipa_ros2_control.repos
rosdep install --from-path src --ignore-src
colcon build
```





## Package layout

- `curt_mini_description` provides models, meshes, Xacro, and simulation configuration without the hardware driver dependency.
- `curt_mini_teleop` provides the joystick launch and configuration.
- `curt_mini_bringup` provides hardware bringup and depends on both packages and `ipa_ros2_control`.

Launch the base with `ros2 launch curt_mini_bringup robot_base.launch.py`. The joystick launch in `curt_mini_bringup` is a wrapper around `curt_mini_teleop`. Older model references to `$(find curt_mini)/models/...` or `package://curt_mini/models/...` must use `curt_mini_description` instead. Simulation configuration lives in `curt_mini_description/config`. Joystick configuration lives in `curt_mini_teleop/config`.

Set `publish_robot_description` to `True` (default value) for standalone base use. If you use the base as part of a bigger robot system (for example an arm on top), the combined bringup should publish the robot description. In this case set `publish_robot_description` to `False`. Only one robot state publisher should own the base model and its TF frames.


The driver launch accepts `controllers_file` for its controller manager configuration; direct invocations default to `curt_mini_bringup/config/ros2_control.yaml`.

## View the robot from another PC

The updated `robot_description` refers to meshes with
`package://curt_mini_description/...` URIs. RViz resolves these URIs on the PC where RViz runs, so install the matching description package there and source its workspace before starting RViz. In this checkout, run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select curt_mini_description
source install/setup.bash
ros2 pkg prefix curt_mini_description
rviz2
```

In RViz, add a RobotModel display, set its Description Topic to
`/robot_description`, and select `base_link` as the Fixed Frame if no odometry frame is available. 

Run the same package version on the robot and the RViz PC.
