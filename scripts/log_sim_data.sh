#!/bin/bash
#This script is used to start all of the different logging scripts used in a test and save the data in an organized fasion in the documents folder

#save args (arg 1 is the name of the level, arg 2 is the name of the robot)
level=$1
robot=$2

#save the directory that the script was run in
original_cd=$PWD

#source the ros setup script and then source the workspace setup script in order to use ros commands
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

echo -----------------------------------------------------------
echo
echo "Starting loggers"
echo
echo -----------------------------------------------------------

#cd to the logs folder inside of resources in the uml_3d_race package
roscd uml_3d_race
cd resources/logs

#create a folder to put the logs into and name the folder using the name of the map being used and the current time
name="$level"_"$robot"_$(date +'%F_%T')
mkdir $name
cd $name

#create a folder for the geotiff maps to live in
mkdir geotiff_maps

#start the logging scripts
roslaunch uml_3d_race geotiff_writer.launch map_dir:=$ROS_WORKSPACE/src/uml_3d_race/resources/logs/$name/geotiff_maps &
pid1=$!
rostopic echo -p sim_log > sim_log.csv &
pid2=$!
rostopic echo -p robot_config > robot_config.csv &
pid3=$!

sleep 5s

#kill the background programs when exiting the script
trap "kill -2 $pid1; kill -2 $pid2; kill -2 $pid3; wait; trap - INT TERM ERR; cd $original_cd" INT TERM ERR