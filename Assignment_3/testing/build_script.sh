#!/bin/bash
source /opt/ros/noetic/setup.bash
cp -r AutSys_Labs_Testframework/Assignment_2/testframework catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/nlopt.git
cd ..
catkin init
catkin build
