#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_goal_publisher");
    ros::NodeHandle nh;

    std::string robot_name;

    float goal_x;
    float goal_y;

    nh.getParam(ros::this_node::getName()+"/robot_name", robot_name);
    nh.getParam(ros::this_node::getName()+"/goal_x", goal_x);
    nh.getParam(ros::this_node::getName()+"/goal_y", goal_y);

    std::string topic_name = robot_name + "/goal_pose";
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 1000, true);

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = topic_name;
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.orientation.w = 1;

    ros::Rate rate(1);
    while(ros::ok()){
        goal_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}