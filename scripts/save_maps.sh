#!/bin/bash
#This script is used to save 2d and 3d maps and organize the maps using folders

#arg 1 is the map name, arg 2 is the boolean for saving a 2d map, and arg 3 is the boolean for saving a 3d map

#first source the ros setup script and then source the workspace setup script in order to use ros commands
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

#check if a 2d map needs to be saved
if $2 ; then
    #first create the folder where the map is going to live
    roscd uml_3d_race
    cd resources/static_maps/2d
    mkdir -p $1

    #then cd to the folder and save the map
    cd $1
    rosrun map_server map_saver -f $1
fi

#check if a 3d map needs to be saved
if $3 ; then
    #first create the folder where the map is going to live
    roscd uml_3d_race
    cd resources/static_maps/3d
    mkdir -p $1

    #then cd to the folder and save the map
    cd $1
    rosrun octomap_server octomap_saver $1.bt
fi