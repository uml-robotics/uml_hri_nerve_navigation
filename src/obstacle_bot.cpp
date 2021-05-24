//Same thing as mover, but it never stops requesting for goals and does not bother with navigating toward its initial goal

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <uml_hri_nerve_navigation/Goal.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::ServiceClient costmap_service;
ros::ServiceClient goal_service;

void goalCallback(uml_hri_nerve_navigation::Goal goal_msg)
{
  move_base_msgs::MoveBaseGoal goal;

  //translate Goal Message to a MoveBaseGoal
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_msg.goal.x;
  goal.target_pose.pose.position.y = goal_msg.goal.y;
  tf2::Quaternion angle;
  angle.setRPY(0, 0, goal_msg.goal.theta);
  goal.target_pose.pose.orientation.z = angle.getZ();
  goal.target_pose.pose.orientation.w = angle.getW();

  //Send the goal to the navigation action client
  ac->sendGoal(goal);
  ROS_INFO("Obstacle bot starting navigation toward goal: %s", goal_msg.description.c_str());

  //Wait for the robot to finish moving
  ac->waitForResult();

  //Clear costmaps once finished navigating
  std_srvs::Empty service_msg;
  if (!costmap_service.call(service_msg))
  {
    ROS_WARN("Failed to clear costmaps");
  }

  //Add a small delay before continuing to navigate
  ros::Duration(2).sleep();

  //Request next goal
  if (!goal_service.call(service_msg))
  {
    ROS_ERROR("Failed to request for a goal");
  }

  return;
}

int main(int argc, char **argv)
{
  // Setup ros node and NodeHandle
  ros::init(argc, argv, "mover_node");
  ros::NodeHandle n;

  //Subscribe to the goal topic
  ros::Subscriber sub = n.subscribe("goal", 10, goalCallback);

  ac = new MoveBaseClient("move_base", true);
  costmap_service = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  goal_service = n.serviceClient<std_srvs::Empty>("get_new_goal");

  //wait for the action server to come up
  while (!ac->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //wait for services
  costmap_service.waitForExistence();

  //Request first goal
  std_srvs::Empty service_msg;
  if (!goal_service.call(service_msg))
  {
    ROS_ERROR("Failed to request for a goal");
  }

  //Keeps the node running and performs necessary updates
  ros::spin();

  return 0;
}