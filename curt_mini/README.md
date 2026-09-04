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
vcs import --recursive src/ < src/curt_mini/curt_mini/curt_mini.repos
vcs import --recursive src/ < src/curt_mini/ipa_ros2_control/ipa_ros2_control.repos
rosdep install --from-path src --ignore-src
colcon build
```




