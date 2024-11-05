#!/bin/bash
source /opt/ros/noetic/setup.bash
pip install em==3.3.4
cp -r AutSys_Labs_Testframework/Assignment_2/testframework catkin_ws/src
cd catkin_ws
catkin init
catkin build -DTEST_FLAG=0
