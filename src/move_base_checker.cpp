#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>

bool robot1_goal_reached = false;
bool robot2_goal_reached = false;

void robot1_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &result){  
    if (!robot1_goal_reached && result->status.status == result->status.SUCCEEDED){
        ROS_INFO("Robot1 has reached its goal");
        robot1_goal_reached = true;
    }
}

void robot2_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &result){   
    if (!robot2_goal_reached && result->status.status == result->status.SUCCEEDED){
        ROS_INFO("Robot2 has reached its goal");
        robot2_goal_reached = true;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_base_checker_node");
    ros::NodeHandle nh;
    ROS_INFO("Checker for move_base tests is running");

    // variables for getting the robots name as parameters from the launch file for having the code to be used by any robot
    std::string robot1_name = "pioneer";
    std::string robot2_name = "pioneer_bot";
    nh.getParam(ros::this_node::getName()+"/robot1_name",robot1_name);
    nh.getParam(ros::this_node::getName()+"/robot2_name",robot2_name);

    std::string robot1_result_topic = robot1_name + "_move_base/result";
    std::string robot2_result_topic = robot2_name + "_move_base/result";

    ros::Subscriber robot1_result_sub = nh.subscribe(robot1_result_topic, 10, robot1_result_callback);
    ros::Subscriber robot2_result_sub = nh.subscribe(robot2_result_topic, 10, robot2_result_callback);
    

    ros::Rate rate(10);
    while(ros::ok()){
        if (robot1_goal_reached && robot2_goal_reached){
            ROS_INFO("Both robots has reached their goals, shutting down the node now....");
            ros::shutdown();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}