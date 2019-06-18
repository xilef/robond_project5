#!/bin/sh
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; roslaunch gmapping slam_gmapping_pr2.launch" &
sleep 5
xterm  -e  " source ~/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
