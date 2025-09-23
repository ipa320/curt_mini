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
git clone -b jazzy-devel git@gitlab.cc-asp.fraunhofer.de:ipa323/robots/curt_mini
chmod +x curt_mini/clone_repos.sh
./curt_mini/clone_repos.sh
cd ..
rosdep install --from-path src --ignore-src --rosdistro jazzy -y -r
colcon build
```




