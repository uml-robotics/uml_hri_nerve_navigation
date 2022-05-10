#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> MBFClient; 
MBFClient* ac;
ros::Publisher goal_pub;
ros::Publisher result_pub;

void callback(const nav_msgs::PathConstPtr& msg){
    ROS_INFO("Path is received");
    mbf_msgs::ExePathGoal target_path_;
    nav_msgs::Path path_;
    geometry_msgs::Pose2D goal;
    std_msgs::Bool result_bool;

    path_ = *msg;

    // Publishing goal
    goal.x = path_.poses[path_.poses.size() - 1].pose.position.x;
    goal.y = path_.poses[path_.poses.size() - 1].pose.position.y;
    goal_pub.publish(goal);

    target_path_.controller = "TrajectoryPlannerROS"; 
    target_path_.path = path_;
    ac->sendGoal(target_path_);
    ac->waitForResult();

    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Path is traversed, goal is reached!");
        result_bool.data = true;
        result_pub.publish(result_bool);
    }
    else if(ac->getState() == actionlib::SimpleClientGoalState::ABORTED) {
        result_bool.data = false;
        result_pub.publish(result_bool);
        ROS_INFO("Goal aborted");
    }
    else { 
        ROS_INFO("Robot failed to move");
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "mbf_mover_node");
    ros::NodeHandle nh;

    std::string robot_name = "pioneer";
    nh.getParam(ros::this_node::getName()+"/robot_name",robot_name);

    std::string action_name = robot_name + "_move_base_flex";
    std::string status_topic = robot_name + "/test_status";
    std::string goal_topic = robot_name + "/goal";
    std::string result_topic = robot_name + "/result";
    std::string path_topic = robot_name + "/path";

    ac = new MBFClient(action_name, true);
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for Move Base server to come up");
    }
    
    
    ros::Publisher test_status_pub = nh.advertise<std_msgs::Bool>(status_topic, 1000, true);
    goal_pub = nh.advertise<geometry_msgs::Pose2D>(goal_topic, 1000, true);
    result_pub = nh.advertise<std_msgs::Bool>(result_topic, 1000, true);
    ros::Subscriber sub = nh.subscribe(path_topic, 1000, callback);

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