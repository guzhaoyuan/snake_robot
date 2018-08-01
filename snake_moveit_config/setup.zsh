#!/bin/zsh
export MYROBOT_NAME="twoJointSnake"
export PLANNING_GROUP="twoJointSnake"
export BASE_LINK="0"
export EEF_LINK="8"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
