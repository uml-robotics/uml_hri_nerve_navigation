/*  
    The goal_pub node publishes the current navigation goal to the goal topic.  In a navigation test, two goals are specified and the
    robot travels between the two goals for a specifed number of iterations.  The goal_pub keeps track which goal is the current goal
    and current goal is swapped if the goal_pub service is called.
*/

#include "ros/ros.h"
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

bool callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
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
  goal_pub = n.advertise<geometry_msgs::Pose2D>("goal", 10, false);

  //Create the opposite publisher object, mainly used by obstacle bot
  goal_pub_opposite = n.advertise<geometry_msgs::Pose2D>("goal_opposite", 10, false);
  
  //Create the service callback
  ros::ServiceServer goal_server = n.advertiseService("get_new_goal", callback);

  //Keeps the node running and perform necessary updates
  ros::spin();

  return 0;
}
