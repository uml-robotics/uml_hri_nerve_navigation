![3D Race Banner](resources/screenshots/repo_banner.png)  
# UML 3D RACE  

## Setup:
1. Clone this repository into your catkin workspace.
  > cd ~/<your_ws>/src   
  > git clone https://github.com/uml-robotics/uml_3d_race.git  
2. Build your catkin workspace.   
  > cd ..  
  > catkin build   
3. Source the workspace.  
  > source devel/setup.bash   
4. Run the setup script for uml_3d_race. (While still in your workspace directory)
  > rosrun uml_3d_race setup.sh  

At this point, if all goes well, the package should be ready to run.  

**NOTE BEFORE STARTING:** Sometimes Gazebo can be a bit finicky, and may fail to launch properly for an array of reasons. If something goes wrong, Ctrl+c and try again a few times. If the problem persists there may be an actual issue that needs to be resolved first, but unless you've already started making changes within this package, that shouldn't be the case.  

## Running a Test:   
1. In a terminal, run the run_test.sh bash script (This will launch all of the required nodes needed to run the test and then perform the test). The bash script can be run with the following arguments:
    * [MAP] - String argument that specifies which environment the robot will navigate in.  A list of avaliable levels can be found below.
    * [SIM] - Boolean argument that specifies whether to launch the test in simulation or not. (true/false)
    * [GUI] - Boolean argument that specifies whether to launch the Gazebo GUI or not. (true/false)
    * [ROBOT] - String argument that specifies which robot to use in the test.
    * [ITERATIONS] - Int Argument that specifies how many times the robot will travel from points A to B and B to A.
    * [OBSTACLE_BOT] - Boolean Argument that specifies whether to spawn a second robot that the other robot has to avoid. (true/false)

    The maps that are able to be launched are the following:
      * level1
      * level4
      * nerve1_base_world
      * nerve1_full_regular
      * nerve1_full_low
      * nerve1_full_high
      * nerve1_full_high_clip
      * nerve1_half_regular
      * nerve1_half_low
      * nerve1_half_high
      * nerve2_base_world
      * nerve2_full_regular
      * nerve2_full_low
      * nerve2_full_high
      * nerve2_half_regular
      * nerve2_half_low
      * nerve2_half_high
      * nerve3_base_world

    The robots that are available to use are the following:
      * fetch
      * pioneer

    The argument order for the bash script are the following
      * run_test.sh [MAP] [SIM] [ROBOT] [ITERATIONS]
      * run_test.sh [MAP] [SIM] [GUI] [ROBOT] [ITERATIONS]
      * run_test.sh [MAP] [SIM] [GUI] [ROBOT] [ITERATIONS] [OBSTACLE_BOT]

    The following command runs a test on level1 in simulation with the fetch robot and the robot performs 10 iterations (Note: This command is ran in the uml_3d_race directory)
      > . run_test.sh level1 true true fetch 10

## Stopping the Test:
If the test runs smoothly, then the bash script will automatically close the test and no action is needed.

If things inevitably go wrong, shutdown the terminal running the bash script with 'Ctrl+c', and start the estop to ensure the robot will stop moving via:  
  > roslaunch uml_3d_race estop.launch  

If in simulation and the robot gets stuck or leaves the desired navigation area, the position of the robot can be reset via:
  > roslaunch uml_3d_race reset_robot.launch  

## Adding a New World:
  TODO

## Adding a New Robot:
  TODO

## File Structure:  
* **launch/** - Contains all necessary ROS .launch files.  
  * **levels/** - Contains all of the .launch files for setting up the various worlds.
  * **navigation/** - Contains all of the .launch files used for navigation.
  * **robots/** - Contains all of the .launch files for setting up the various robots.  The files with the spawn_ prefix deal with spawning and setting up robots in simuation and the files with the run_ prefex deal with setting up physical robots.
* **msg/** - Custom ROS message definition.  
* **resources/** - Package resources.  
  * **config/** - Contains the configurations for all of the robots.  Each robot has its own subfolder.  
  * **logs/** - Contains the logs from each performed test.  Subfolders are used to distinguish the logs from different tests.
  * **models/** - Contains package models.  Individual robot models and definitions are found in this folder as well
    * **obstacles/** - Contains obstacle models for making new worlds.   
    * **track_models/** - Contains track models for making new worlds.
* **screenshots/** - Contains screenshots from various tests performed
  * **static_maps/** - Contains maps to be used for navigation
    * **2d/** - Contains 2d maps used by map_server
    * **3d/** - Contains octomaps used by octomap_server   
  * **textures/** - Contains custom Gazebo texture files.  
* **src/** - Contains all c++ source files.  
  * **estop.cpp** - Publishes zero velocity on the /cmd_vel topic at a fast rate in case of an emergency
  * **goal_pub.cpp** - Publishes the A and B goal point on the /goal and /goal_opposite topics and alternates the goal points after the robot reaches the goal it was navigating to.
  * **mover.cpp** - Reads the current goal on the /goal topic and commands the navigation stack to move to that goal.  
  * **obstacle_bot.cpp** - Commands the obstacle bot to move to various goals.
  * **referee.cpp** - Logs various robot data used to evaluate navigation tests. 
  * **reset_robot.cpp** - Subscribes to the /spawn topic and sets the robot models position.  
  * **spawn_pub.cpp** - Publishes the robots spawn location on the /spawn topic.   
  * **goal_pub.cpp** - Publishes a goal on the /goal topic.
* **worlds/** - Contains all .world files for Gazebo to load.  
* **run_test.sh** - A script that automatically launches and performs a test
* **setup.sh** - A setup script that makes the models and textures provided in this package available for Gazebo to use.  

## Important Launch Files:    
* **estop.launch** - A software emergency stop in case the robot gets out of control during a test.
  >roslaunch uml_3d_race estop.launch

* **rviz.launch** - Launches RVIZ with a default configuration that displays most of the important components used in the tests.  Each robot has a unique RVIZ configuration and the desired robot configuration is specified in the launch command.
  >roslaunch uml_3d_race rviz.launch robot:=pioneer

* **spawn_world.launch** - Launches Gazebo as an empty world or loads a specific world file. This launch file does not include any robots. This file is included in other launch files to reduce their complexity, and it can be useful for creating or modifying world files.  
To launch an empty world:  
  > roslaunch uml_3d_race gazebo.launch load_world:=false  

  To specify a world:  
    > roslaunch uml_3d_race gazebo.launch world_path:=[world path starting after the worlds folder excluding the .world extension]  

* **setup_robot.launch** - Sets up the robot that is being used in the test.  If the test is in simulation and Gazebo is running, the robot will be spawned into the Gazebo environment, otherwise the necessary setup to run a physical robot will be performed.  This launch file includes a launch file inside of the /launch/robots folder depending on which robot is being used and whether the test is in simulation or not. 
To spawn a robot with a specified name and/or position (example):  
  > roslaunch uml_3d_race spawn_robot.launch robot:=pioneer x:=5.0 y:=5.0 yaw:=3.14 level:=level1 sim:=true

* **setup_test.launch** - Includes the spawn_world.launch and spawn_robot.launch in order to load a gazebo world if needed, and spawn a robot within it in just one file. The world file and robot spawn positions are sent to their respective launch files as arguments. You will notice that each level launch file simply includes this launch file and specifies the relevant arguments, making it easy to launch specific configurations. Changing the default values of the arguments in this file is perfectly acceptable, but you can specify them in a command as well.  
For example:  
  > roslaunch uml_3d_race setup_test.launch world_name:=level1 world_file:=level1 spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.0 spawn_yaw:=0.0 sim:=true robot:=pioneer

* **race.launch** - This launch file simply starts a referee node and a mover node simultaneously. The mover node is responsible for commanding the robot between goals A and B for a specified # of iterations and the logger node is responsible for publishing all of the robot data log during the test.
  > roslaunch uml_3d_race race.launch iterations:=5

* **geotiff_writer.launch** - This launch file is responsible for logging the robots postion on a map over the course of the test. The logger will by default save maps to a /geotiff_maps folder inside of the resources folder.
  > roslaunch uml_3d_race geotiff_writer.launch
## Useful Resources:  
[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)  
[Gazebo Tutorials](http://gazebosim.org/tutorials)  
[ROS .launch file documentation](http://wiki.ros.org/roslaunch/XML)  
[ROS Navigation](http://wiki.ros.org/navigation)  
[Navigation Tuning Guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)    
[Nav Tuning Guide Paper](http://kaiyuzheng.me/documents/navguide.pdf)   

[URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)   
[URDF XML Specification](https://wiki.ros.org/urdf/XML)   
[SDF (Simulation Description Format) Specification for Gazebo Models](http://sdformat.org/spec)  
[OGRE Material Script Documentation (Gazebo Textures)](https://ogrecave.github.io/ogre/api/1.12/_material-_scripts.html)
