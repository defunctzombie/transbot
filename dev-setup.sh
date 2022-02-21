#!/bin/bash

source /opt/ros/noetic/setup.bash
cd workspace
catkin_make
source devel/setup.bash