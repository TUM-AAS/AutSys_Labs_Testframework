#!/bin/bash
source /opt/ros/noetic/setup.bash
cp -r AutSys_Labs_Testframework/Assignment_1/testframework catkin_ws/src
ls catkin_ws/src
cd catkin_ws
catkin init
catkin build

