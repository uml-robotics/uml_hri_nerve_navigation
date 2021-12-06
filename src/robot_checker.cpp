#include <ros/ros.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

using namespace std;

int seconds = 0;

vector<nav_msgs::Odometry> robot_odoms;
vector<geometry_msgs::PoseStamped> robot_goals;
vector<string> robot_headers;
vector<string> robots_stuck;
vector<nav_msgs::Odometry> prev_odom_readings;
vector<string> robot_names;

bool kill_node = false;
bool swapped = false;
int counter = 0;

void goal_callback(geometry_msgs::PoseStamped goal_msg){
    bool goal_found = false;
    if (counter < robot_names.size()) {
        string temp_topic_name = robot_names[counter] + "/goal_pose";
        for (int i = 0; i < robot_goals.size(); ++i) {
            if (robot_goals[i].header.frame_id == goal_msg.header.frame_id) {
                goal_found = true;
                break;
            }
        }
        if (!goal_found && temp_topic_name == goal_msg.header.frame_id) {
            robot_goals.push_back(goal_msg);
        }
        counter++;
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
    cout << "Here goalsReached " << endl;
    bool goal_reached = false;
    if (odom_vector.size() <= 1){
        return;
    }
    else{
        cout << "ELSE" << endl;
        for(int i = 0; i < odom_vector.size(); ++i){
            cout  << "ODOM: " << odom_vector[i].pose.pose.position.y << " :::: GOAL: " << goal_vector[i].pose.position.y << endl;
            if (abs(odom_vector[i].pose.pose.position.x - goal_vector[i].pose.position.x) < 0.4
             && abs(odom_vector[i].pose.pose.position.y - goal_vector[i].pose.position.y) < 0.4) {
                cout << "HERE IN THIS LOOP" << endl;
                    robot_headers.push_back(odom_vector[i].header.frame_id);
            }
        }
    }
}

void checkRobotsStuck (vector<nav_msgs::Odometry> odom_vector){
    bool robots_stuck = false;
    if (prev_odom_readings.size() == 0){
        for (int i = 0; i < odom_vector.size(); ++i){
            prev_odom_readings.push_back(odom_vector[i]);
        }
    }
    else{
        for (int i = 0; i < prev_odom_readings.size(); ++i){
            if (abs(prev_odom_readings[i].pose.pose.position.x - odom_vector[i].pose.pose.position.x) < 0.1
            &&  abs(prev_odom_readings[i].pose.pose.position.y - odom_vector[i].pose.pose.position.y) < 0.1){
                string temp;
                stringstream ss(odom_vector[i].header.frame_id);
                getline(ss, temp, '/');
                cout << seconds << " seconds passed, still no goals received for " << temp << endl;
                robots_stuck = true;
            }
            prev_odom_readings[i] = odom_vector[i];
        }
        if (robots_stuck && seconds == 45){
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
        double time_elapsed_sec = time_elapsed/1e9;
        if (time_elapsed_sec <= 60){
            minutes = 0;
            seconds = time_elapsed_sec;
        }
        else{
            minutes = time_elapsed_sec/60;
            seconds = (int)time_elapsed_sec % 60;
        }
    }
}

int main(int argc, char** argv){
    bool odoms_subscribed = false;
    ros::init(argc, argv, "robot_checker_node");
    ros::NodeHandle nh;

    auto start = std::chrono::steady_clock::now();

    thread time_counter_thread(time_counter, start);
    string some_string = "pioneer,pioneer_bot";
    vector<string> odom_topic_names;
    vector<string> goal_topic_names;

    vector<ros::Subscriber> odom_subs;
    vector<ros::Subscriber> goal_subs;

    stringstream ss(some_string);
    while(ss.good()){
        string temp;
        getline(ss, temp, ',');
        robot_names.push_back(temp);
    }

    // creating odom topic names
    for (int i = 0; i < robot_names.size(); ++i){
        odom_topic_names.push_back(robot_names.at(i) + "/odom");
    }

    // creating goal topic names
    for (int i = 0; i < robot_names.size(); ++i){
        goal_topic_names.push_back(robot_names.at(i) + "/goal_pose");
    }
    
    // subscribing
    odom_subs.resize(odom_topic_names.size());
    goal_subs.resize(goal_topic_names.size());
    for (int i = 0; i < robot_names.size(); ++i){
        //cout << "Printing topics: " << odom_topic_names[i] << ", " << goal_topic_names[i] << endl;
        odom_subs[i] = nh.subscribe(odom_topic_names[i], 1, odom_callback);
        goal_subs[i] = nh.subscribe(goal_topic_names[i], 1, goal_callback);
    }

     

    ros::Rate rate(1);
    while(ros::ok()){
        if (!odoms_subscribed && robot_odoms.size() == odom_topic_names.size()){
            time_counter_thread.detach();
            odoms_subscribed = true;
        }

        if(kill_node){
            cout << "ROBOTS STUCK, SHUTTING DOWN THE NODE...................." << endl;
            ros::shutdown();
        }
        checkGoalsReached(robot_odoms, robot_goals);
        
        if (robot_headers.size() == robot_names.size()){
            cout << "ROBOTS REACHED THEIR GOALS, SHUTTING DOWN THE NODE...................." << endl;
            ros::shutdown();
        }
        if (odoms_subscribed && seconds % 15 == 0){
            checkRobotsStuck(robot_odoms);
            cout << endl;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}