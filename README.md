# Home Service Bot

Submission for Project 5 of the Udacity Robotics Software Engineer Nanodegree Program. This repository contains a robot that will follow the sequence below:

1. Go to a start goal designated by an orange marker.
2. Remove the marker then wait 5 seconds to simulate an object pick up.
3. Go to an end goal then show the marker again to simulate the object drop off.

Navigation is achieved using the [move_base](http://wiki.ros.org/move_base) package and sending navigation goals

For localization and mapping there are 2 versions available:
* Using the [AMCL](http://wiki.ros.org/amcl) package to perform localization with the map provided by [slam_gmapping](https://github.com/ros-perception/slam_gmapping.git)
* Using the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) package to perform SLAM (currently a work in progress)

The `add_markers` package is a service that handles the marker setting and hiding.

The `pick_objects` package is a node that handles the goal setting for the robot.

## How to run

### Setup
Make sure you have gazebo and rviz installed and you have setup a catkin workspace

Clone the repo to your catkin workspace's src folder

eg:
`git clone https://github.com/xilef/robond_project5 /home/robond/workspace/catkin_ws/src
`

Go to the root of your catkin workspace and run `catkin_make` to build the whole repo

### Execute

To modify the start and end goal positions, open the launch file:

`pick_objects/launch/pick_objects.launch`

And modify the X, Y or W params with the prefix startGoal* or endGoal*.

Once satisfied with the goal positions run the script:

`scripts/home_service.sh`

The script will run the necessary nodes in sequence.

## License

The content in this repository is free to use.
