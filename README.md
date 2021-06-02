![Navigation Banner](resources/screenshots/nist1.png)  
# UML HRI Nerve Navigation

## About:
  The UML HRI Nerve Navigation package is used for performing several navigation tests that help quantify important characteristics of a navigating robot. Examples of these characteristics include the performance of a robot's path planner and the peformance of a robot's obstacle detection algorithms. A navigation test consists of an environment with a set of goals. The robot will start the test at a starting goal and then navigate to the rest of the goals in sequential order.  A single iteration of a navigation test will be completed once the robot navigates back to the starting goal. To measure the repeatability of the robot's performance, a full navigation test will consist of multiple iterations of the robot traveling to all of the goals and then back to the starting goal. After the test is complete, characteristics of a navigating robot can be found by comparing the robot's performace to the theoretical performance or the performace of a difference robot that completed  the same test. Depending on the configuration of the environement, different characteristics can be more easily measured than others.  For example, if one wanted to measure how well a robot can avoid small obstacles, small cubes can be inserted into the navigation environment.  The image below shows an example of a navigation test environment with two goals, A and B.
  
  ![Example Test Environment](resources/screenshots/test_setup.png) 

## Package Setup:
1. Clone this repository into your catkin workspace.
  > cd ~/<your_ws>/src   
  > git clone https://github.com/uml-robotics/uml_hri_nerve_navigation.git 
2. Install package dependencies
  > rosdep update   
  > rosdep install uml_hri_nerve_navigation
3. Build your catkin workspace.   
  > cd ..  
  > catkin build   
4. Source the workspace.  
  > source devel/setup.bash
5. If simulated tests are going to be performed, the UML HRI Nerve Nav Sim Resources package is also required.  To install, repeat the same commands above but replace the git clone command and the rosdep install command to the following:
  > git clone https://github.com/uml-robotics/uml_hri_nerve_nav_sim_resources.git  
  > rosdep install uml_hri_nerve_nav_sim_resources
6. Also, If simulated tests are going to be performed, a setup bash script must be ran in the UML HRI Nerve Nav Sim Resources package before running anything
  > roscd uml_hri_nerve_nav_sim_resources    
  > . setup.sh

At this point, if all goes well, the package should be ready to run.  

**NOTE BEFORE STARTING:** Sometimes Gazebo can be a bit finicky, and may fail to launch properly for an array of reasons. If something goes wrong, Ctrl+c and try again a few times. If the problem persists there may be an actual issue that needs to be resolved first.

## Running a Test:   
There are two main ways to run a test:
1. Launch the desired level launch file with the start_test argument set to true. By setting the start_test argument to true, the robot will automatically start performing a navigation test and will stop running once the test finishes. 

    The following command runs a test on level1 in simulation with the fetch robot and the robot performs 10 iterations
    > roslaunch uml_hri_nerve_navigation level1.launch start_test:=true sim:=true robot:=fetch iterations:=10

2. Launch the desired level launch file with the start_test argument set to false. By setting the start_test argument to false, the robot will not start navigating automatically, but you can control when the robot starts navigating and also perform multiple tests in the same Gazebo environment. To start a test, launch the start_test.launch file.

    The following command sets up a test on level4 in simulation with the pioneer robot.
    > roslaunch uml_hri_nerve_navigation level4.launch start_test:=false sim:=true robot:=fetch

    Then following command starts a navigation test with 20 iterations.
    > roslaunch uml_hri_nerve_navigation start_test.launch start_test:=true sim:=true robot:=fetch iterations:=10   

**Note:** If you running a physical test, the amcl node will initialize to the coordinate (0,0) which is probably not where the robot is actually located. To fix this, launch a world file with start_test set to false and before starting the test, use the approximate position tool in RViz to initialize the robot to the correct location.

## Defining Test Goals:
The robots can be configured to navigate to multiple goal locations in a given test.  To set the goals for a navigation test:
1. Create a folder in the goals folder and name the folder whatever the world_name argument is in the desired environment's launch file
2. Create two CSV files, the first one called goals.csv and the second one called obstacle_bot_goals.csv.  Goals.csv defines goals for the main test robot to follow and obstacle_bot_goals.csv defines goals for the obstacle bot to follow.
3. Launch the desired world with the start_test argument set to false and launch RViz. Then, in another terminal, run a rostopic echo on the /clicked_point topic. Using the publish point tool, click desired goal locations on the map and record the coordinates for the point that are displayed in the terminal running the rostopic echo command.
4. In the CSV files, column 1 defines the goal names, column 2 defines the x coordinates, column 3 defines the y coordinates, and column 4 defines the goal yaws. When the goal_publisher node reads the file, the first row is ignored, so feel free to add headers for the columns or just leave the first row blank.  Rows 2 and on will define a new goal for the robot to navigate to. The robot will navigate to the goals from top to bottom in the order defined by the CSV.  Thus, the starting goal for the test will be the goal defined in row 2 of the CSV file.
5. If you need an example to refer to, look at the maps that already have goals defines such as nerve1_base_world.


## Stopping the Test:
If the level launch file is run with the start_test argument set to true, then the launch file will automatically stop running once the test finishes.  If the level launch file is run with the start_test argument set to false, the start_test launch file will automatically stop running once the test finishes, but the level launch file needs to be closed manually with 'Ctrl+c'.

If things inevitably go wrong, shutdown all the launch files with 'Ctrl+c', and start the estop to ensure the robot will stop moving via:  
  > roslaunch uml_hri_nerve_navigation estop.launch  

If in simulation and the robot gets stuck or leaves the desired navigation area, the position of the robot can be reset via:
  > roslaunch uml_hri_nerve_navigation reset_robot.launch  

## Adding a New Navigation Environment:
1. **Creating a Gazebo world in the UML HRI Nerve Nav Sim Resources package if the environment is going to be simulated**  
For more information on adding a simulated world to the UML HRI Nerve Nav Sim Resources package, refer to the UML HRI Nerve Nav Sim Resources documentation
2. **Creating a navigation map**    
Launch the setup_test.launch file specifing the world_path if in simulation and set the navigation argument to false.  By setting the navigation argument to false, the robot will launch all of the necessary nodes to perform navigation mapping.  The robot will automatically generate a map by using the robot's sensor data, but the robot must move throughout the entire environment in order to map the entire environemnt.  To move the robot, follow the terminal instructions to move the robot around using a keyboard.  When mapping, it is recommended to launch RViz to view the current map and ensure that there are not any chuncks missing from the map.
3. **Save the maps after mapping the entire environment**    
Launch the save_maps.launch file specifing the map_name argument as the name for the current environment.
4. **Defining test goals**    
To define goals for a test inside a given environment, follow the instructions provided under the Defining Test Goals section.

## Adding a New Robot:
  TODO

## File Structure:  
* **goals/** - Defines all of the navigation goals for each environment.
* **launch/** - Contains all necessary ROS .launch files.  
  * **levels/** - Contains all of the .launch files for setting up the various navigation environments.
  * **navigation/** - Contains all of the .launch files used for navigation.
  * **robots/** - Contains all of the .launch files for setting up the various robots.
* **msg/** - Custom ROS message definition.  
* **resources/** - Package resources.  
  * **config/** - Contains the configurations for all of the robots.  Each robot has its own subfolder.  
  * **logs/** - Contains the logs from each performed test.  Subfolders are used to distinguish the logs from different tests.
  * **screenshots/** - Contains screenshots from various tests performed as well as visuals for documentation
  * **static_maps/** - Contains maps to be used for navigation
    * **2d/** - Contains 2d maps used by map_server
    * **3d/** - Contains octomaps used by octomap_server    
* **src/** - Contains all c++ source files.  
  * **dord_calculator.cpp** - Calculates and logs the DORD metric on the nerve_long_hall map
  * **estop.cpp** - Publishes zero velocity on the /cmd_vel topic at a fast rate in case of an emergency
  * **goal_pub.cpp** - Reads the navigation goals defined in goal CSV files and publishes the current navigation goal on the /goal topic.  Also provides services to be able to change the current goal and reset the goal order.
  * **logger.cpp** - Logs various robot data used to evaluate navigation tests.
  * **mover.cpp** - Reads the current goal on the /goal topic and commands the navigation stack to move to that goal.  
  * **obstacle_bot.cpp** - Commands the obstacle bot to move to various goals.
  * **setup_fetch.cpp** - Resets Fetch's current state into a state that is state for navigation - TODO
* **run_test.sh** - A script that automatically launches and performs a test
## Important Launch Files:    
* **[INSERT DESIRED WORLD].launch** - This launch includes either the run_test.launch file or the setup_test.launch file depending on the value of the start_test argument.  This launch file also sets all of the map dependent variables such as the map names and the two test goal locations.   
**Arguments:**
  * **start_test (default: true)** - Boolean argument that specifies whether the robot should automatically start moving or not.  Note that specifing the iterations and clear_costmaps argument is unnessecary if start_test is false.
  * The rest of the arguments are the same as the run_test.launch and the setup_test.launch files   

  Example command to launch level4 in simulation with the fetch robot, start_test set to true, and the robot will perform 5 iterations.
  > roslaunch uml_hri_nerve_navigation level4.launch sim:=true robot:=fetch start_test:=true iterations:=5

* **run_test.launch** - A high level launch file that includes setup_test and the mover node to run a full complete navigation test. This allows for a complete test to be launched with a single launch file.    
**Arguments:**
  * **iterations (default: 1)** - Int argument that specifies the number of times the robot will travel from goal A to goal B and back to goal A.
  * **clear_costmaps (default: true)** - Boolean argument that specifies whether the robot's costmaps will be cleared in between each navigation goal.
  * The rest of the arguments are the same as the setup_test.launch file

  Example command to launch nerve1_base_world in simulation with the pioneer robot.  The pioneer will then perform a navigation test with 6 iterations.
  > roslaunch uml_hri_nerve_navigation run_test.launch sim:=true robot:=pioneer world_name:=nerve1_base_world world_path:= nerve1/nerve1_base_world iterations:=6

* **setup_test.launch** - A high level launch file that includes all of the necessary launch files to set up an environment and the robot for a navigation test.  The launch files included are a level launch file in the Nav Sim Resources package if in simulation, start_loggers.launch, and setup_[ROBOT].launch.  This launch file also starts the goal_publisher node for managing the test goals.  This launch file also includes all of the arguments required by all of the launch files and nodes.     
**Arguments:**
  * **sim (default: true)** - Int argument that specifies the number of times the robot will travel from goal A to goal B and back to goal A.
  * **gui (default: true)** - Boolean argument that specifies whether the robot's costmaps will be cleared in between each navigation goal.
  * **navigate (default: true)** - Boolean argument that specifies whether the robot should launch in navigation mode or in mapping mode.
  * **static_map (default: true)** - Boolean argument that specifies whether the robot will use a static global map while navigating or use slam.
  * **3d (default: true)** - Boolean argument that specifies whether the robot should navigate with 3d sensors (rgbd cameras) or just with 2d sensors (LIDARs).
  * **robot (default: pioneer)** - String argument that specifies which robot to set up and spawn in if in simulation.
  * **world_name** - String argument that specifies the name of the static navigation map for the current environment.
  * **world_path** - String argument that specifies the file location of the .world file to set up in Gazebo relative to the uml_hri_nerve_nav_sim_resources/worlds folder.
  * **obstacle_bot (default: false)** - Boolean argument that specifes whether to set up an obstacle_bot or not
  * **dynamic_obstacle (default: false)** - Boolean argument that specifies whether to spawn in a dynamic obstacle or not if in simulation.
  * **obstacle_name (default: caution_100cm_block)** - String argument that specifes the name of the obstacle folder in the models/obstacles folder to spawn in as the dynamic obstacle if in simulation.
  * **flip_obstacle (default: caution_100cm_block)** - Boolean argument that specifies whether the dynamic obstacle should spawn in at pose 1 or pose 2 if in simulation.

  Example command to set up a test in level1 in simulation with a pioneer robot.
  > roslaunch uml_hri_nerve_navigation setup_test.launch world_name:=level1 world_file:=level1 spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.0 spawn_yaw:=0.0 sim:=true robot:=pioneer

* **start_test.launch** - This launch file simply starts a mover node. The mover node is responsible for commanding the robot between goals A and B for a specified number of iterations. The robot's costmap_clear service call between iterations can also be turned on and off using the boolean parameter clear_costmaps. An obstacle bot mover node will also be started if the obstacle bot parameter is true  
**Arguments:** 
  * **iterations (default: 1)** - Int argument that specifies the number of times the robot will travel from goal A to goal B and back to goal A.
  * **clear_costmaps (default: true)** - Boolean argument that specifies whether the robot's costmaps will be cleared in between each navigation goal.
  * **obstacle_bot (default: false)** - Boolean argument that specifes whether an obstacle_bot is being used or not

  Example command for starting a test with 5 iterations, and the robot's costmaps will clear in between goals
  > roslaunch uml_hri_nerve_navigation start_test.launch iterations:=5 clear_costmaps:=true

* **estop.launch** - A software emergency stop in case the robot gets out of control during a test.       
**Arguments:**
  * **namespace (default: "")** - String argument that specifies the namespace the estop node should run in.

  Example command for activing the estop for a robot that does not have a namespace
  >roslaunch uml_hri_nerve_navigation estop.launch

* **rviz.launch** - Launches RViz with a default configuration that displays most of the important components used in the tests.  Each robot has a unique RViz configuration and the desired robot configuration is specified in the launch command.    
**Arguments:**
  * **robot (default: "")** - String argument that specifies which robot config to launch. If the argument is empty, a default RViz config will launch
  * **config_file (default: "config")** - String argument that specifies the name of the config files.  

  Example command for launching RViz for the pioneer robot
  >roslaunch uml_hri_nerve_navigation rviz.launch robot:=pioneer

* **setup_<Desired_Robot>.launch** - Sets up the robot that is being used in the test.  If the test is in simulation and Gazebo is running, the robot will be spawned into the Gazebo environment, otherwise the necessary setup to run a physical robot will be performed.  This launch file includes a launch file inside of the /launch/robots folder depending on which robot is being used and whether the test is in simulation or not.     
**Arguments:**
  * **level (default: true)** - String argument that specifies the name of the static navigation map for the current environment.
  * **navigate (default: true)** - Boolean argument that specifies whether the robot should launch in navigation mode or in mapping mode.
  * **3d (default: true)** - Boolean argument that specifies whether the robot should navigate with 3d sensors (rgbd cameras) or just with 2d sensors (LIDARs).

  Example command to set up a pioneer robot with a spawn location for amcl localization
  > roslaunch uml_hri_nerve_navigation spawn_robot.launch robot:=pioneer x:=5.0 y:=5.0 yaw:=3.14 level:=level1

* **start_loggers.launch** - This launch file is responsible for logging various properties of a robot throughout a navigation test such as position, current goal, distance to goal, etc. The logger will also save geotiff_maps created by the hector_geotiff package throughout the test.  The logs are sorted out by the time, current robot, and current environment.  The logs can be found in the resources/logs folder.   
**Arguments:**
  * **level** - String argument that specifies the name of the current environment. Used for naming the log files.
  * **robot** - String argument that specifies the current robot used in the test. Used for naming the log files.

  Example command to start logging a test that is being performed in level1 and with the fetch robot
  > roslaunch uml_hri_nerve_navigation start_loggers.launch level:=level1 robot:=fetch

* **save_maps.launch** - This launch file is responsible for saving the maps created by gmapping and octomap_server. To create a map, refer to the Adding a New Navigation Environment section above.  The maps are saved to the resources/static_maps folder and are seperated by whether the map is created using gmapping or octomap_server.   
**Arguments:**
  * **map_name** - String argument that specifies the name of the current environment. Used for naming the log files.
  * **2d (default: true)** - Boolean argument that specifies whether to save the 2d map created by gmapping or not.
  * **3d (default: true)** - Boolean argument that specifies whether to save the 3d map created by octomap_server or not.

  Example command to start logging a test that is being performed in level1 and with the fetch robot
  > roslaunch uml_hri_nerve_navigation start_loggers.launch level:=level1 robot:=fetch

## Available Arguments
The maps that are able to be launched are the following:
  * level1
  * level2 (2d map only)
  * level3 (2d map only)
  * level4
  * nerve_long_hall
  * nerve_physical (no simulated world)
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
  * mir (2d navigation only)

## Useful Resources:  
[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)  
[ROS .launch file documentation](http://wiki.ros.org/roslaunch/XML)  
[ROS Navigation](http://wiki.ros.org/navigation)  
[Navigation Tuning Guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)    
[Nav Tuning Guide Paper](http://kaiyuzheng.me/documents/navguide.pdf)   