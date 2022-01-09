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

//template<class T>
bool checkGreaterOdom(nav_msgs::Odometry first, nav_msgs::Odometry last){
    return (first.header.frame_id < last.header.frame_id);
}

bool checkGreaterGoal(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped last){
    return (first.header.frame_id < last.header.frame_id);
}

bool kill_node = false;
bool swapped = false;
int counter = 0;

void goal_callback(geometry_msgs::PoseStamped goal_msg){
    bool goal_found = false;
    for (int i = 0; i < robot_goals.size(); ++i) {
        if (robot_goals[i].header.frame_id == goal_msg.header.frame_id) {
            //robot_goals[i] = goal_msg;
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

void checkGoalsReached(vector<nav_msgs::Odometry> odom_vector, vector<geometry_msgs::PoseStamped> goal_vector){
    bool found;
    if (robots_reached.size() == odom_vector.size()){
        kill_node = true;
    }
    for(int i = 0; i < odom_vector.size(); ++i){
        found = false;
        // cout  << "ODOM_X: " << odom_vector[i].pose.pose.position.x << " :::: GOAL: " << goal_vector[i].pose.position.x << endl;
        // cout  << "ODOM_Y: " << odom_vector[i].pose.pose.position.y << " :::: GOAL: " << goal_vector[i].pose.position.y << endl;
        if (abs(odom_vector[i].pose.pose.position.x - goal_vector[i].pose.position.x) < 0.4
         && abs(odom_vector[i].pose.pose.position.y - goal_vector[i].pose.position.y) < 0.4) {
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
            cout << "REACHED_SIZE: " << robots_reached.size() << endl;
        }
    }
}

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

    thread time_counter_thread(time_counter, start);
    string robot_names_str = "pioneer,pioneer_bot";

    nh.getParam(ros::this_node::getName()+"/robot_names",robot_names_str);

    string odom_topic_name = " ";
    string goal_topic_name = " ";
    vector<string> odom_topic_names;
    vector<string> goal_topic_names;

    vector<ros::Subscriber> odom_subs;
    vector<ros::Subscriber> goal_subs;

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
        if (!vectors_sorted && robot_odoms.size() == odom_subs.size()){
            time_counter_thread.detach();
            sort(robot_odoms.begin(), robot_odoms.end(), checkGreaterOdom);
            sort(robot_goals.begin(), robot_goals.end(), checkGreaterGoal);
            vectors_sorted = true;
        }
        else if (vectors_sorted){
            checkGoalsReached(robot_odoms, robot_goals);
            if (seconds != 0 && seconds % 15 == 0){
                checkRobotsStuck(robot_odoms, robot_goals);
            }
            if (kill_node){
                ros::shutdown();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}