#!/bin/sh
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; rosrun add_markers add_markers_orig"
