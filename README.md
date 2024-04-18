# Curt Mini

Select a branch and the ROS version for your robot. You may need a ROS1 and a ROS2 workspace.

## General Notes

This repository is used for setting up and starting the CURTmini software stack.
It consists of the configurations and dependencies for the sensor equipment on the robot.
In the bringup folder you find the launchfiles for starting the base and the whole navigation.

Currently outdoor navigation is running packages in ros1 and ros2 simultaneously so you will need 3 workspaces to run the complete robot.
1. ROS1 workspace - checkout the corresponding branch of this repo and the selected robot to setup the workspace
2. ROS2 workspace - checkout the corresponding branch of this repo and the selected robot to setup the workspace
3. ros1_bridge workspace - this ws is built with ROS2 but sources ROS1 and ROS2 for running. See https://github.com/ros2/ros1_bridge


## Setting up the foxy workspace

```
mkdir -p <colcon_ws>/src
cd <colcon_ws>/src 
git clone -b foxy-devel git@gitlab.cc-asp.fraunhofer.de:ipa323/robots/curt_mini
chmod +x curt_mini/clone_repos.sh
./curt_mini/clone_repos.sh
cd ..
rosdep install --from-path src --ignore-src --rosdistro foxy -y -r
colcon build
```




