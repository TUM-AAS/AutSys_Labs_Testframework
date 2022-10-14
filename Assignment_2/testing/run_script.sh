#!/bin/bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
source devel/setup.bash
roslaunch testframework test.launch
cd ..
file="catkin_ws/results.txt"
echo "\n"
if [ -f "$file" ]
then
	cat $file
else
	echo "No results available."
fi
cd AutSys_Labs_Testframework/Assignment_2/testing
mkdir build
cd build
cmake ..
make
./AutSys_Labs_Testframework/Assignment_2/testing/build/runTest