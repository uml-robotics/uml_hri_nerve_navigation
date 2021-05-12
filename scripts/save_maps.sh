#!/bin/bash
#This script is used to save 2d and 3d maps and organize the maps using folders

#arg 1 is the map name, arg 2 is the boolean for saving a 2d map, and arg 3 is the boolean for saving a 3d map

#first source the ros setup script and then source the workspace setup script in order to use ros commands
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

#save the original directory so at the end the original directory can be returned to
original_dir=$PWD

#go to the package directory
roscd uml_hri_nerve_navigation

#go to the maps folder
cd resources/static_maps

#check if a 2d map needs to be saved
if $2 ; then
    cd 2d
    rosrun map_server map_saver -f $1
    cd ..
fi

#check if a 3d map needs to be saved
if $3 ; then
    cd 3d
    rosrun octomap_server octomap_saver $1.bt
    cd ..
fi

#cd to original directory
cd $original_dir