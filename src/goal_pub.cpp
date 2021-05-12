/*  
    The goal_pub node publishes the current navigation goal to the goal topic.  In a navigation test, two goals are specified and the
    robot travels between the two goals for a specifed number of iterations.  The goal_pub keeps track which goal is the current goal
    and current goal is swapped if the goal_pub service is called.
*/

#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher goal_pub;
ros::Publisher goal_pub_opposite;
int counter;
float goal_x;
float goal_y;
float goal_theta;
float spawn_x;
float spawn_y;
float spawn_theta;
float goal_tolerance;

bool newGoalCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  geometry_msgs::Pose2D goal;
  geometry_msgs::Pose2D goal_opposite;

  //Determines which goal needs to be published
  if(counter % 2 == 0)
  {
    //Set the goal to the original goal coords
    goal.x = goal_x;                  
    goal.y = goal_y;
    goal.theta = goal_theta;

    //Set the opposite goal to the original spawn coords
    goal_opposite.x = spawn_x;                  
    goal_opposite.y = spawn_y;
    goal_opposite.theta = spawn_theta;
  }
  else
  {
    //Set the goal to the original spawn coords
    goal.x = spawn_x;                  
    goal.y = spawn_y;
    goal.theta = spawn_theta;

    //Set the opposite goal to the original goal coords
    goal_opposite.x = goal_x;                  
    goal_opposite.y = goal_y;
    goal_opposite.theta = goal_theta;
  }

  //Publish the goal
  goal_pub.publish(goal);

  //Publish the opposite goal
  goal_pub_opposite.publish(goal_opposite);

  //Increase count to alternate goals
  counter++;
  
  return true;
}

bool initialGoalCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  geometry_msgs::Pose2D startingPosition;
  geometry_msgs::Pose2D startingGoal;

  //Set the initial Goal
  startingGoal.x = goal_x;                  
  startingGoal.y = goal_y;
  startingGoal.theta = goal_theta;

  //Set the initial position
  startingPosition.x = spawn_x;                  
  startingPosition.y = spawn_y;
  startingPosition.theta = spawn_theta;

  //Publish the initial position on the goal topic so the robot navigates to the initial position
  goal_pub.publish(startingPosition);

  //Publish the initial goal
  goal_pub_opposite.publish(startingGoal);
  
  return true;
}

bool resetGoalsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  //Reset counter so the next time get_new_goal is called, the goal topic will publish the goal parameters
  counter = 0;

  return true;
}

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;

  //Set default values
  counter = 0;
  goal_x = 0.0;
  goal_y = 0.0;
  goal_theta = 0.0;
  spawn_x = 0.0;
  spawn_y = 0.0;
  spawn_theta = 0.0;

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/goal_x",goal_x);
  n.getParam(ros::this_node::getName()+"/goal_y",goal_y);
  n.getParam(ros::this_node::getName()+"/goal_theta",goal_theta);
  n.getParam(ros::this_node::getName()+"/spawn_x",spawn_x);
  n.getParam(ros::this_node::getName()+"/spawn_y",spawn_y);
  n.getParam(ros::this_node::getName()+"/spawn_theta",spawn_theta);

  //Display node information on startup
  ROS_INFO("GOAL_A | Point: (%.2f,%.2f)\tGOAL_B | Point: (%.2f,%.2f)",spawn_x,spawn_y,goal_x,goal_y);

  //Create the publisher object
  goal_pub = n.advertise<geometry_msgs::Pose2D>("/goal", 1, true);

  //Create the opposite publisher object, mainly used by obstacle bot
  goal_pub_opposite = n.advertise<geometry_msgs::Pose2D>("goal_opposite", 10, false);
  
  //Create the service callbacks
  ros::ServiceServer goal_server = n.advertiseService("get_new_goal", newGoalCallback);
  ros::ServiceServer initial_goal_server = n.advertiseService("get_initial_goal", initialGoalCallback);
  ros::ServiceServer reset_goal_server = n.advertiseService("reset_goals", resetGoalsCallback);

  //Keeps the node running and perform necessary updates
  ros::spin();

  return 0;
}
