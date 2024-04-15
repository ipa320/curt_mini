#!/bin/sh
# get curt mini repo
vcs import < curt_mini/ipa_outdoor_robot_bringup.repos
# get all nav packages
vcs import < ipa_outdoor_navigation/ipa_outdoor_navigation.repos
# get necessary hardware packages
vcs import < ipa_ros2_control/ipa_ros2_control.repos

# install cheetah library
echo "Do you want to make install mini-cheetah-tmotor-can?"
select yn in "Yes" "No"; do
  case $yn in
    Yes ) cd mini-cheetah-tmotor-can; mkdir build; cmake ..; make; sudo make install; break;;
    No ) exit;;
  esac
done