#!/bin/bash

# display urdf in rviz and using gui joint_publisher to control each joint of robot
roslaunch snake_robot display.launch model:='$(find snake_robot)/snake_description/urdf/twoJoints.urdf'

# display robot with joint_publisher make robot go ellipse
# roslaunch snake_robot display_twoJointsSnake.launch