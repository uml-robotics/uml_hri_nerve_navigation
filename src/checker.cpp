/*
    The checker is created to support the python API for running automated test. Pupose of this test is to shutdown the node if the robots has reached their goal or are stuck.
*/

#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <unistd.h>

// booleans for checking if the robots reached their goals
bool robot1_goal_reached = false;
bool robot2_goal_reached = false;

// variables for storing the goal positions
geometry_msgs::Pose robot1_goal;
geometry_msgs::Pose robot2_goal;

std::vector<geometry_msgs::Point> robot1_poses;
int robot1_index = 0;
std::vector<geometry_msgs::Point> robot2_poses;
int robot2_index = 0;
bool kill_node = false;

int r1_counter = 0;
int r2_counter = 0;

// callback funtion for goals topic 
void goals_callback(const tuw_multi_robot_msgs::RobotGoalsArrayConstPtr& goals_msg){
    // variable for storing the goal 
    tuw_multi_robot_msgs::RobotGoalsArray goals;
    goals = *goals_msg;
    
    // storing the goals for each robots 
    robot1_goal = goals.robots[0].destinations[0];
    robot2_goal = goals.robots[1].destinations[0];
}

// callback function for robot1 odom topic
void robot1_odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
    // variable for storing robot1 odom information
    nav_msgs::Odometry robot1_odom;
    robot1_odom = *odom_msg;
    
    if (robot1_goal.position.x != 0 && robot1_goal.position.y != 0){
        robot1_poses.push_back(robot1_odom.pose.pose.position);
        
        if (!robot1_goal_reached && robot1_poses.size() % 15 == 0){
            if (abs(robot1_poses.at(robot1_index).x - robot1_poses.at(robot1_poses.size() - 1).x) < 0.5 &&
                abs(robot1_poses.at(robot1_index).y - robot1_poses.at(robot1_poses.size() - 1).y) < 0.5){
                ROS_INFO("Robot1 has not moved from past %d seconds", robot1_poses.size());
                robot1_index = robot1_poses.size();
                if (robot1_poses.size() == 45){
                    kill_node = true;
                }
            }
            else{
                robot1_index = 0;
                for (int i = 0; i < robot1_poses.size(); ++i){
                    robot1_poses.pop_back();
                }
            }
        }
    }
    else {
        r1_counter++;
        if (r1_counter != 0 && r1_counter % 15 == 0){
            ROS_INFO("No goals sent, please send a goal for robot1 to move....");
        }
    }

    // checks if the difference between robots current position and goal position is not much, then the goal is reached
    if (abs(robot1_goal.position.x - robot1_odom.pose.pose.position.x) < 0.5 && 
        abs(robot1_goal.position.y - robot1_odom.pose.pose.position.y) < 0.5 &&
        robot1_goal.position.x != 0 && robot1_goal.position.y != 0 && !robot1_goal_reached){
        ROS_INFO("Robot 1 has reached its goal!");
        robot1_goal_reached = true;
    }
    else if (robot1_goal_reached); // for avoiding printing the message from the if condition continously
}

// callback function for robot2 odom topic
void robot2_odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
    // variable for storing robot2 odom information
    nav_msgs::Odometry robot2_odom;
    robot2_odom = *odom_msg;
    if (robot2_goal.position.x != 0 && robot2_goal.position.y != 0){
        robot2_poses.push_back(robot2_odom.pose.pose.position);
        
        if (!robot2_goal_reached && robot2_poses.size() % 15 == 0){
            if (abs(robot2_poses.at(robot2_index).x - robot2_poses.at(robot2_poses.size() - 1).x) < 0.5 &&
                abs(robot2_poses.at(robot2_index).y - robot2_poses.at(robot2_poses.size() - 1).y) < 0.5){
                ROS_INFO("Robot2 has not moved from past %d seconds", robot2_poses.size());
                robot2_index = robot2_poses.size();
                if (robot2_poses.size() == 45){
                    kill_node = true;
                }
            }
            else{
                robot2_index = 0;
                for (int i = 0; i < robot2_poses.size(); ++i){
                    robot2_poses.pop_back();
                }
            }
        }
    }
    else {
        r2_counter++;
        if (r2_counter != 0 && r2_counter % 15 == 0){
            ROS_INFO("No goals sent, please send a goal for robot2 to move....");
        }
    }

    // checks if the difference between robots current position and goal position is not much, then the goal is reached
    if (abs(robot2_goal.position.x - robot2_odom.pose.pose.position.x) < 0.5 && 
        abs(robot2_goal.position.y - robot2_odom.pose.pose.position.y) < 0.5 &&
        robot2_goal.position.x != 0 && robot2_goal.position.y != 0 && !robot2_goal_reached){
        ROS_INFO("Robot 2 has reached its goal!");
        robot2_goal_reached = true;
    }
    else if (robot2_goal_reached); // for avoiding printing the message from the if condition continously
}

int main(int argc, char** argv){
    ros::init(argc, argv, "checker_node");
    ros::NodeHandle nh;
    ROS_INFO("MBF Checker is running");

    // strings created for getting the robots name as parameters from the launch file so that the code can be used with any two robots
    std::string robot1_name = "pioneer";
    std::string robot2_name = "pioneer_bot";
    nh.getParam(ros::this_node::getName()+"/robot1_name",robot1_name);
    nh.getParam(ros::this_node::getName()+"/robot2_name",robot2_name);

    // strings for adding robot namespace before their odom topics in order to be subscribed
    std::string robot1_odom_topic = robot1_name + "/odom";
    std::string robot2_odom_topic = robot2_name + "/odom";

    // subscribers
    ros::Subscriber goals_sub = nh.subscribe("/goals", 1000, goals_callback);
    ros::Subscriber robot1_odom_sub = nh.subscribe(robot1_odom_topic, 1, robot1_odom_callback);
    ros::Subscriber robot2_odom_sub = nh.subscribe(robot2_odom_topic, 1, robot2_odom_callback);

    ros::Rate rate(1);
    while(ros::ok()){
        // checks if both robots have reached their goals then prepare for shutting down the node
        if (robot2_goal_reached){
            ROS_INFO("Both robots has reached their goals, shutting down the node....");
            if (robot1_poses.size() != 0){
                robot1_poses.clear();
            }
            if (robot2_poses.size() != 0){
                robot2_poses.clear();
            }
            sleep(5);
            ros::shutdown();
        }
        if (kill_node){
            ROS_INFO("One (or both) of the robots still haven't moved, shuting down the node....");
            robot1_poses.clear(); 
            robot2_poses.clear();
            ros::shutdown();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}