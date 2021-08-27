#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tuw_goal_sender_node");
    ros::NodeHandle nh;

    float r1_goal_x = 6.0;
    float r1_goal_y = 14.0;

    float r2_goal_x = 4.0;
    float r2_goal_y = 12.0;

    nh.getParam(ros::this_node::getName()+"/r1_goal_x",r1_goal_x);
    nh.getParam(ros::this_node::getName()+"/r1_goal_y",r1_goal_y);

    nh.getParam(ros::this_node::getName()+"/r2_goal_x",r2_goal_x);
    nh.getParam(ros::this_node::getName()+"/r2_goal_y",r2_goal_y);


    ros::Publisher goals_pub = nh.advertise<tuw_multi_robot_msgs::RobotGoalsArray>("/goals", 1000, true);

    tuw_multi_robot_msgs::RobotGoalsArray goals;

    goals.header.frame_id = "map";
    goals.header.stamp = ros::Time::now();
    goals.header.seq = 0;

    goals.robots.clear();
    goals.robots.resize(2);
    
    goals.robots[0].robot_name = "pioneer";

    goals.robots[0].destinations.clear();
    goals.robots[0].destinations.resize(1);


    goals.robots[0].destinations[0].position.x = r1_goal_x;
    goals.robots[0].destinations[0].position.y = r1_goal_y;

    goals.robots[0].destinations[0].position.z = 0.0;
    goals.robots[0].destinations[0].orientation.x = 0.0;
    goals.robots[0].destinations[0].orientation.y = 0.0;
    goals.robots[0].destinations[0].orientation.z = 0.0;
    goals.robots[0].destinations[0].orientation.w = 1.0;

    goals.robots[1].robot_name = "pioneer_bot";
    
    goals.robots[1].destinations.clear();
    goals.robots[1].destinations.resize(1);
    
    goals.robots[1].destinations[0].position.x = r2_goal_x;
    goals.robots[1].destinations[0].position.y = r2_goal_y;

    goals.robots[1].destinations[0].position.z = 0.0;
    goals.robots[1].destinations[0].orientation.x = 0.0;
    goals.robots[1].destinations[0].orientation.y = 0.0;
    goals.robots[1].destinations[0].orientation.z = 0.0;
    goals.robots[1].destinations[0].orientation.w = 1.0;

    goals_pub.publish(goals);

    ROS_INFO("Goal is published");
    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}