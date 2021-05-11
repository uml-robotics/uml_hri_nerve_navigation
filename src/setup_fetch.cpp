#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/PointHeadGoal.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> HeadClient;

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "setup_fetch");
    ros::NodeHandle n;

    HeadClient head = actionlib::SimpleActionClient("head_controller/point_head", true);
    head.waitForServer();

    control_msgs::PointHeadActionGoal goal;
    goal.goal.pointing_axis ;






    
