/*
    The robot checker node contains the code which compares the robots odom and the goal positions for determining if the robots reached their goals. The robot checker also
    determines if the robots havent moved for some amount of seconds (currently set to 45). If the robots reaches the goal or if the robots havent moved for 45 seconds, then
    the robot checker node would shut itself down.
*/

#include <ros/ros.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>

using namespace std;

int seconds = 0;

vector<nav_msgs::Odometry> robot_odoms;
vector<geometry_msgs::PoseStamped> robot_goals;
vector<string> robots_reached;
vector<string> robots_stuck;
vector<nav_msgs::Odometry> prev_odom_readings;
vector<string> robot_names;

// functions for sorting the odom and goal_pose vectors
bool checkGreaterOdom(nav_msgs::Odometry first, nav_msgs::Odometry last){
    return (first.header.frame_id < last.header.frame_id);
}

bool checkGreaterGoal(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped last){
    return (first.header.frame_id < last.header.frame_id);
}

bool kill_node = false;
bool swapped = false;
int counter = 0;

// callbacks
void goal_callback(geometry_msgs::PoseStamped goal_msg){
    bool goal_found = false;
    for (int i = 0; i < robot_goals.size(); ++i) {
        if (robot_goals[i].header.frame_id == goal_msg.header.frame_id) {
            goal_found = true;
            break;
        }
    }
    if (!goal_found){
        robot_goals.push_back(goal_msg);
    }
}

void odom_callback(nav_msgs::Odometry msg){
    bool odom_found = false;
    for (int i = 0; i < robot_odoms.size(); ++i){
        if (robot_odoms[i].header.frame_id == msg.header.frame_id){
            robot_odoms[i] = msg;
            odom_found = true;
            break;
        }
    }
    if (!odom_found){
        robot_odoms.push_back(msg);
    }
}

// functions which compares the odom with goal position for checking if the robots reached their goals
void checkGoalsReached(vector<nav_msgs::Odometry> odom_vector, vector<geometry_msgs::PoseStamped> goal_vector){
     cout << "CHECKER_NODE: COMPARING TWO POSITIONS" << endl;
     ROS_WARN("CHECKER_NODE: COMPARING TWO POSITIONS");

    bool found;
    if (robots_reached.size() == odom_vector.size()){
        kill_node = true;
    }
    for(int i = 0; i < odom_vector.size(); ++i){

        cout << "CHECKER_NODE: ODOM: " << odom_vector[i].pose.pose.position.x << ", GOAL: " << goal_vector[i].pose.position.x << endl;

        found = false;
        if (abs(odom_vector[i].pose.pose.position.x - goal_vector[i].pose.position.x) <= 0.65
         && abs(odom_vector[i].pose.pose.position.y - goal_vector[i].pose.position.y) <= 0.65
         && abs(odom_vector[i].pose.pose.orientation.w - goal_vector[i].pose.orientation.w) < 0.02) {
            for (int j = 0; j < robots_reached.size(); ++j){
                if (robots_reached[j] == odom_vector[i].header.frame_id){
                    found = true;
                }
            }
            if (!found){
                string temp;
                stringstream ss(odom_vector[i].header.frame_id);
                getline(ss, temp, '/');
                cout << temp << " successfully reached its goal...." << endl;
                robots_reached.push_back(odom_vector[i].header.frame_id);
            }
        }
    }
}

// function for checking if the robots havent moved for 45 seconds (if they havent reached their goals already)
void checkRobotsStuck (vector<nav_msgs::Odometry> odom_vector, vector<geometry_msgs::PoseStamped> goal_vector){
    bool robots_stuck = false;
    if (prev_odom_readings.size() == 0){
        for (int i = 0; i < odom_vector.size(); ++i){
            prev_odom_readings.push_back(odom_vector[i]);
        }
    }
    else{
        for (int i = 0; i < prev_odom_readings.size(); ++i){
            if (abs(prev_odom_readings[i].pose.pose.position.x - odom_vector[i].pose.pose.position.x) < 0.1
            &&  abs(prev_odom_readings[i].pose.pose.position.y - odom_vector[i].pose.pose.position.y) < 0.1
            && (abs(odom_vector[i].pose.pose.position.x - goal_vector[i].pose.position.x) > 0.4
             || abs(odom_vector[i].pose.pose.position.y - goal_vector[i].pose.position.y) > 0.4)){
                string temp;
                stringstream ss(odom_vector[i].header.frame_id);
                getline(ss, temp, '/');
                cout << seconds << " seconds passed, " << temp << " hasnt moved" << endl;
                robots_stuck = true;
            }
            prev_odom_readings[i] = odom_vector[i];
        }
        if (robots_stuck && seconds >= 45){
            cout << "Shutting down the node due to robot(s) being stuck" << endl;
            kill_node = true;
            return;
        }
    }
}

// function for another thread for getting the time
void time_counter(chrono::steady_clock::time_point start){
    fstream outFile;
    while(true) {
        sleep(1);
        auto end = std::chrono::steady_clock::now();
        double time_elapsed = double(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        int minutes;
        seconds = time_elapsed/1e9;
    }
}

int main(int argc, char** argv){
    bool vectors_sorted = false;
    ros::init(argc, argv, "robot_checker_node");
    ros::NodeHandle nh;

    auto start = std::chrono::steady_clock::now();

    // another thread is started for counting time
    thread time_counter_thread(time_counter, start);
    string robot_names_str = "pioneer,pioneer_bot";

    nh.getParam(ros::this_node::getName()+"/robot_names",robot_names_str);

    string odom_topic_name = " ";
    string goal_topic_name = " ";
    vector<string> odom_topic_names;
    vector<string> goal_topic_names;

    vector<ros::Subscriber> odom_subs;
    vector<ros::Subscriber> goal_subs;

    // the robot names string is seperated into robot namespaces under the loop, also odoms and goal_poses are subscribed
    stringstream ss(robot_names_str);
    while(ss.good()){
        string temp;
        getline(ss, temp, ',');
        robot_names.push_back(temp);
        odom_topic_name = temp + "/odom";
        goal_topic_name = temp + "/goal_pose";
        odom_subs.push_back(nh.subscribe(odom_topic_name, 1, odom_callback));
        goal_subs.push_back(nh.subscribe(goal_topic_name, 1, goal_callback));
    }

    ros::Rate rate(1);
    while(ros::ok()){
        // if the odoms and goal_pose are subscribed then the odom and goal_pose vector are sorted (makes it easy for comparing)
        if (!vectors_sorted && robot_odoms.size() == odom_subs.size()){
            time_counter_thread.detach();
            sort(robot_odoms.begin(), robot_odoms.end(), checkGreaterOdom);
            sort(robot_goals.begin(), robot_goals.end(), checkGreaterGoal);
            vectors_sorted = true;
        }
        else if (vectors_sorted){
            checkGoalsReached(robot_odoms, robot_goals);
            // every 15 seconds check if the robots are stuck
            if (seconds != 0 && seconds % 15 == 0){
                checkRobotsStuck(robot_odoms, robot_goals);
            }
            // the kill_node boolean is true then the node should shut down
            if (kill_node){
                ros::shutdown();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}