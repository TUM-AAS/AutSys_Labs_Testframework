#!/bin/bash
source /opt/ros/noetic/setup.bash
cp -r AutSys_Labs_Testframework/Assignment_3/testframework catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git
cd ..
catkin init
catkin build -DTEST_FLAG=0
