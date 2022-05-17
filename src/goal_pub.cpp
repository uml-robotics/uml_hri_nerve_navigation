/*  
    The goal_pub node publishes the current navigation goal to the goal topic.  In a navigation test, a set of navigation goals are specified in a csv file and the
    robot travels between the specified goals for a specifed number of iterations.  The goal_pub keeps track which goal is the current goal and the next goal is recieved
    if the goal_pub service is called.  This node also handles parsing the goal csv file.  For more info on the defining goals, check out the README.
*/

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <uml_hri_nerve_navigation/Goal.h>
#include <boost/algorithm/string.hpp>
#include <stdexcept>

class GoalManager
{
private:
  std::vector<uml_hri_nerve_navigation::Goal> goalArray;
  std::ifstream csvFile;

  ros::Publisher goal_pub;
  int counter;

  void parseCsvFile()
  {
    std::string csvLine;

    //Read the line since the header is useless
    std::getline(csvFile, csvLine);

    //The all of the following lines and split each line by the commas
    while (std::getline(csvFile, csvLine))
    {
      //Make sure that the line read is not empty and causes the split function call to cause an error
      if (!csvLine.empty())
      {
        //seperates the lines by the commas
        std::vector<std::string> data;
        boost::algorithm::split(data, csvLine, boost::is_any_of(","));

        //parse the seperated strings into the goal msg
        uml_hri_nerve_navigation::Goal goal;
        goal.description = data[0];
        goal.goal.x = std::stod(data[1].c_str());
        goal.goal.y = std::stod(data[2].c_str());
        goal.goal.theta = std::stod(data[3].c_str());

        ROS_INFO("Goal: %s registered at x:%f, y:%f and yaw:%f", goal.description.c_str(), goal.goal.x, goal.goal.y, goal.goal.theta);

        //Add the goal to the goal vector
        goalArray.push_back(goal);
      }
    }
  }

public:
  GoalManager(ros::NodeHandle n)
  {
    //Get file location
    std::string file_location;
    n.getParam(ros::this_node::getName() + "/file_path", file_location);

    //Open the file
    csvFile.open(file_location.c_str());

    //Check if the file is valid
    if (!csvFile)
    {
      ROS_ERROR("Unable to open goal file");
      throw std::invalid_argument("Invalid file name");
    }

    //parse the file
    parseCsvFile();

    //Close the file
    csvFile.close();

    //After parsing all of the goals, set the total number of goals inside all of the goal msgs
    for(int i = 0; i < goalArray.size(); i++)
    {
      goalArray[i].total_goals = goalArray.size();
    }

    //Create the publisher object
    goal_pub = n.advertise<uml_hri_nerve_navigation::Goal>("goal", 1, true);

    //Set counter to equal 1 so the first goal published by publishNextGoal is not the spawn location
    counter = 1;
  }

  //Publishes the next goal in the goal array. The goals wrap around to the beginning of the array after the last goal is published
  void publishNextGoal()
  {
    //Determine what is the current goal
    int currentGoalIndex = counter % goalArray.size();

    //Publish the goal
    goal_pub.publish(goalArray[currentGoalIndex]);
    ROS_INFO("Published goal: %s", goalArray[currentGoalIndex].description.c_str());

    //Increase count to get the next goal
    counter++;
  }

  //Publishes the first goal in the goal array
  void publishInitialGoal()
  {
    //Publish the goal
    goal_pub.publish(goalArray[0]);
    ROS_INFO("Published initial goal: %s", goalArray[0].description.c_str());
  }

  //Resets the counter back to 1
  void resetGoals()
  {
    counter = 1;
    ROS_INFO("Goal publisher order reset to beginning");
  }
};

GoalManager *goalManager;

//Service Callbacks

bool newGoalCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  goalManager->publishNextGoal();
  return true;
}

bool initialGoalCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  goalManager->publishInitialGoal();
  return true;
}

bool resetGoalsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  goalManager->resetGoals();
  return true;
}

int main(int argc, char **argv)
{
  //Create ros node
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;
    ROS_ERROR("GOAL PUB IS RUNNING");
  //create a goalManager object
  goalManager = new GoalManager(n);

  //Create the service callbacks
  ros::ServiceServer goal_server = n.advertiseService("get_new_goal", newGoalCallback);
  ros::ServiceServer initial_goal_server = n.advertiseService("get_initial_goal", initialGoalCallback);
  ros::ServiceServer reset_goal_server = n.advertiseService("reset_goals", resetGoalsCallback);

  //Keeps the node running and perform necessary updates
  ros::spin();

  //Delete the goalManager object
  delete goalManager;

  ROS_WARN("object deleted");

  return 0;
}
