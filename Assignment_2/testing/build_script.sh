#!/bin/bash
source /opt/ros/noetic/setup.bash
cp -r AutSys_Labs_Testframework/Assignment_2/testframework catkin_ws/src
cp AutSys_Labs_Testframework/Assignment_2/testing/autograding.json .github/classroom
cd catkin_ws/src
git clone https://github.com/ethz-asl/mav_comm.git
cd ..
catkin init
catkin build -DTEST_FLAG=0
