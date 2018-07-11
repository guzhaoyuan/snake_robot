#!/bin/bash
roslaunch urdf_tutorial display.launch model:='$(find snake_robot)/snake_description/urdf/twoJoints.urdf'
