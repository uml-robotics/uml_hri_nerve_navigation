#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <sstream>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tuw_goal_sender_node");
    ros::NodeHandle nh;

    int num_robots = 0;
    string robot_names_str = " ";
    string robot_goals_str = " ";

    vector<string> robot_names;
    vector<pair<double, double>> robot_goals;

    nh.getParam(ros::this_node::getName()+"/num_robots",num_robots);
    nh.getParam(ros::this_node::getName()+"/robot_names",robot_names_str);
    nh.getParam(ros::this_node::getName()+"/robot_goals",robot_goals_str);

    stringstream ss(robot_names_str);
    while(ss.good()){
        string temp;
        getline(ss, temp, ',');
        robot_names.push_back(temp);
        cout << "TEMP: " << temp;
    }

    stringstream ss1(robot_goals_str);
    while(ss1.good()){
        string temp;
        pair<double, double> temp_pair;
        getline(ss1, temp, ',');
        temp_pair.first = stod(temp);
        getline(ss1, temp, ',');
        temp_pair.second = stod(temp);
        robot_goals.push_back(temp_pair);
    }

    for(auto each:robot_goals){
        cout << "(" << each.first << ", " << each.second << ")" << endl;
    }
    
    ros::Publisher goals_pub = nh.advertise<tuw_multi_robot_msgs::RobotGoalsArray>("/goals", 1000, true);

    tuw_multi_robot_msgs::RobotGoalsArray goals;

    goals.header.frame_id = "map";
    goals.header.stamp = ros::Time::now();
    goals.header.seq = 0;
    goals.robots.clear();
    goals.robots.resize(num_robots);
    for (int i = 0; i < goals.robots.size(); ++i){
        goals.robots[i].robot_name = robot_names[i];
        goals.robots[i].destinations.clear();
        goals.robots[i].destinations.resize(1);

        goals.robots[i].destinations[0].position.x = robot_goals[i].first;
        goals.robots[i].destinations[0].position.y = robot_goals[i].second;
        goals.robots[i].destinations[0].position.z = 0.0;
        goals.robots[i].destinations[0].orientation.x = 0.0;
        goals.robots[i].destinations[0].orientation.y = 0.0;
        goals.robots[i].destinations[0].orientation.z = 0.0;
        goals.robots[i].destinations[0].orientation.w = 1.0;

    }
 
    goals_pub.publish(goals);

    ROS_INFO("Goal is published");
    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}