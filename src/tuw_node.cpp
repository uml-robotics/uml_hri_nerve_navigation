#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

ros::Publisher goal_pub;

void path_callback(const nav_msgs::PathConstPtr& msg){
    ROS_INFO("Path is received");
    nav_msgs::Path path_;
    geometry_msgs::Pose2D goal;

    path_ = *msg;

    // Publishing goal
    goal.x = path_.poses[path_.poses.size() - 1].pose.position.x;
    goal.y = path_.poses[path_.poses.size() - 1].pose.position.y;
    goal_pub.publish(goal);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tuw_node");
    ros::NodeHandle nh;

    std::string robot_name = "pioneer";
    nh.getParam(ros::this_node::getName()+"/robot_name",robot_name);

    std::string status_topic = robot_name + "/test_status";
    std::string goal_topic = robot_name + "/goal";
    std::string path_topic = robot_name + "/path";

    ros::Publisher test_status_pub = nh.advertise<std_msgs::Bool>(status_topic, 1000, true);
    goal_pub = nh.advertise<geometry_msgs::Pose2D>(goal_topic, 1000, true);
    
    ros::Subscriber path_sub = nh.subscribe(path_topic, 1000, path_callback);

    std_msgs::Bool boolean;
    boolean.data = true;
    test_status_pub.publish(boolean);

    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}