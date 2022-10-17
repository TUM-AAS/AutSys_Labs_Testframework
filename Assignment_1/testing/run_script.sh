#!/bin/bash
if [ $1 -eq 1 ]
then
	source /opt/ros/noetic/setup.bash
	cd catkin_ws
	source devel/setup.bash
	roslaunch testframework test.launch
	cd ../AutSys_Labs_Testframework/Assignment_1/testing
	mkdir build
	cd build
	cmake ..
	make
	cd ../../../..
fi
file="catkin_ws/results.txt"
echo ""
echo ""
if [ -r "$file" ]
then
	cat $file
else
	echo "No results available."
fi
./AutSys_Labs_Testframework/Assignment_1/testing/build/runTest --gtest_filter=$2
