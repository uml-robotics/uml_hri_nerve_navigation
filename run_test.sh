#!/bin/bash

#This script launches, executes, and terminates all programs needed to run a navigation test

#save the directory that the script was run in
original_cd=$PWD

#source the ros setup script and then source the workspace setup script in order to use ros commands
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

# Determine which arguments correspond to each variable based on the the total amount of arguments entered
if [[ $# -eq 4 ]]; then
    level=$1
    sim=$2
    gui="true"
    robot=$3 
    threeD="true" 
    clear_costmaps="true"
    iterations=$4
    obstacle_bot="false"
elif [[ $# -eq 5 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4
    threeD="true"
    clear_costmaps="true"
    iterations=$5
    obstacle_bot="false"
elif [[ $# -eq 6 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4
    threeD=$5
    clear_costmaps="true"
    iterations=$6
    obstacle_bot="false"
elif [[ $# -eq 7 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4
    threeD=$5
    clear_costmaps=$6
    iterations=$7
    obstacle_bot="false"
elif [[ $# -eq 8 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4
    threeD=$5
    clear_costmaps=$6
    iterations=$7
    obstacle_bot=$8
fi

# Launch a world in Gazebo
echo -----------------------------------------------------------
echo
echo "Setting up the $robot robot in the $level environment"
echo
echo -----------------------------------------------------------
roslaunch uml_3d_race "$level".launch sim:="$sim" gui:="$gui" navigate:=true robot:="$robot" 3d:="$threeD" obstacle_bot:="$obstacle_bot" &
pid1=$!

sleep 12s

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
pid2=$!
rostopic echo -p sim_log > sim_log.csv &
pid3=$!
rostopic echo -p robot_config > robot_config.csv &
pid4=$!

sleep 5s

echo -----------------------------------------------------------
echo
echo "Starting test"
echo
echo -----------------------------------------------------------
roslaunch uml_3d_race race.launch iterations:="$iterations" clear_costmaps:="$clear_costmaps"
pid5=$!

#this will kill the script if any errors occur during the test
trap "kill -2 $pid5; kill -2 $pid4; kill -2 $pid3; kill -2 $pid2; kill -2 $pid1; wait; trap - INT TERM ERR; cd $original_cd" INT TERM ERR

#once the script raches this point, the race launch file has finished running

echo -----------------------------------------------------------
echo
echo "The test has finished, killing all of the test processes"
echo
echo -----------------------------------------------------------

#kill all of the background processes
kill -2 $pid4
kill -2 $pid3
kill -2 $pid2
kill -2 $pid1
wait

#Reset trap
trap - INT TERM ERR

#finally cd to the original directory the script was run in
cd $original_cd