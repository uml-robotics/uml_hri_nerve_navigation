#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// global variables
ros::Publisher new_odom_pub;
double offset_x, offset_y;

// odom callback
void odom_callback(nav_msgs::Odometry msg) {
    nav_msgs::Odometry new_msg;
    new_msg = msg;
    new_msg.pose.pose.position.x += offset_x;
    new_msg.pose.pose.position.y += offset_y;
    ROS_INFO("NEW ODOM POSITION: %f, %f", new_msg.pose.pose.position.x, new_msg.pose.pose.position.y);
    new_odom_pub.publish(new_msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "rosaria_odom_filter");
    ros::NodeHandle nh;
    std::string initial_topic_name, filtered_topic_name;

    nh.getParam(ros::this_node::getName()+"/offset_x",offset_x);
    nh.getParam(ros::this_node::getName()+"/offset_y",offset_y);
    nh.getParam(ros::this_node::getName()+"/odom_topic",initial_topic_name);
    nh.getParam(ros::this_node::getName()+"/filtered_odom_topic",filtered_topic_name);

    new_odom_pub = nh.advertise<nav_msgs::Odometry>(filtered_topic_name, 1000, true);

    ros::Subscriber odom_sub = nh.subscribe(initial_topic_name, 1000, odom_callback);

    ros::Rate loop_rate(1);
    while(ros::ok()) { 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}