#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <chrono>
#include <unistd.h>
#include <thread>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>

using namespace std;
using namespace nlohmann;

double xOdom, yOdom;
double xGoal, yGoal;
// required for computing distance between two coordinates - for logger (remove?)
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

// global variables for keeping track of whether files are launched
bool gazebo_active = false;
bool map_active = false;
bool tuw_active = false;
map<string, bool> odom_map;
map<string, bool> goal_pose_map;
string tester_status = "";
ros::Publisher tester_status_pub; 

// callbacks
void tester_callback(std_msgs::String msg){
    tester_status = msg.data;
}
void gazebo_callback(gazebo_msgs::ModelStates msg){
    gazebo_active = true;
}
void map_callback(nav_msgs::OccupancyGrid msg){
    map_active = true;
}
void odom_callback(nav_msgs::Odometry msg){
    string temp;
    stringstream ss(msg.header.frame_id);
    getline(ss, temp, '/');
    if (odom_map.find(temp) != odom_map.end() && odom_map[temp]) { return; }
    odom_map[temp] = true;
}
void goal_pub_callback(geometry_msgs::PoseStamped msg){
    string temp;
    stringstream ss(msg.header.frame_id);
    getline(ss, temp, '/');
    if (goal_pose_map.find(temp) != goal_pose_map.end() && goal_pose_map[temp]) { return; }
    goal_pose_map[temp] = true;
}
void tuw_callback(tuw_multi_robot_msgs::RobotInfo msg){
    tuw_active = true;
}

// would be called by a seperate thread
void tester_status_subscriber(vector<string> robot_names){
    vector<ros::Subscriber> odom_subs, goal_pub_subs;
    odom_subs.resize(robot_names.size());
    goal_pub_subs.resize(robot_names.size());

    ros::NodeHandle nh;

    // publisher for publishing the tester status (continue, stop, etc.) (remove? ask Peter?)
    tester_status_pub = nh.advertise<std_msgs::String>("tester_status", 1000, true);

    ros::Subscriber tester_status_sub = nh.subscribe("tester_status", 1000, tester_callback);
    ros::Subscriber gazebo_sub = nh.subscribe("gazebo/model_states", 1000, gazebo_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1000, map_callback);
    ros::Subscriber tuw_sub = nh.subscribe("robot_info", 1000, tuw_callback);

    // initializing the ros publisher and subscribers
    for (int i = 0; i < robot_names.size(); ++i){
        odom_map[robot_names[i]] = false;
        goal_pose_map[robot_names[i]] = false;
        string topic_name = robot_names[i] + "/odom";
        odom_subs[i] = nh.subscribe(topic_name, 1000, odom_callback);
        topic_name = robot_names[i] + "/goal_pose";
        goal_pub_subs[i] = nh.subscribe(topic_name, 1000, goal_pub_callback);
    }

    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

// data structure for a robot
struct robot{
    string robot;
    string robot_namespace;
    double starting_position_x;
    double starting_position_y;
    double goal_position_x;
    double goal_position_y;
    string nav_config;
};
typedef struct robot Robot;

class TestRunner{
 public:
    TestRunner(json json_file){
        map = json_file.at("map").at("map_name");
        world_name = json_file.at("map").at("world_name");
        number_of_robots = json_file.at("robots").at("number_of_robots");
        robots.resize(number_of_robots);
        string robot = "robot_";
        for (int i = 0, j = 1; j < number_of_robots + 1; ++i, ++j){
            string robot_name = robot + to_string(j);
            robots[i].robot = json_file.at("robots").at(robot_name).at("robot");
            robots[i].robot_namespace = json_file.at("robots").at(robot_name).at("robot_namespace");

            robots[i].starting_position_x = json_file.at("robots").at(robot_name).at("starting_position")[0];
            robots[i].starting_position_y = json_file.at("robots").at(robot_name).at("starting_position").at(1);

            robots[i].goal_position_x = json_file.at("robots").at(robot_name).at("goal_position")[0];
            robots[i].goal_position_y = json_file.at("robots").at(robot_name).at("goal_position").at(1);

            robots[i].nav_config = json_file.at("robots").at(robot_name).at("nav_config");
        }
        for (int i = 0; i < robots.size(); ++i){
            robot_names_str += robots[i].robot_namespace;
            robot_goals_str += to_string(robots[i].goal_position_x) + "," + to_string(robots[i].goal_position_y);
            if (i != robots.size() - 1){
                robot_names_str += ",";
                robot_goals_str += ",";
            }
        }
        xGoal = robots[0].goal_position_x;
        yGoal = robots[0].goal_position_y;
    }

    void run(int num_repetitions, bool simulation, bool auto_reset) {
        for (int i = 0; i < num_repetitions; ++i) {
            string out = "  &";
            if (simulation) {
                string launch_gazebo = "roslaunch uml_hri_nerve_nav_sim_resources spawn_world.launch world_path:=" + world_name + ".world" + out;
                system(launch_gazebo.c_str());
                while(!gazebo_active);
                string launch_spawn_robot;
                for (int i = 0; i < number_of_robots; ++i){
                    launch_spawn_robot = "roslaunch uml_hri_nerve_nav_sim_resources spawn_pioneer.launch x:=" + 
                                    to_string(robots[i].starting_position_x) + " y:=" + to_string(robots[i].starting_position_y) + 
                                    " model_type:=" + robots[i].robot_namespace + out;
                    system(launch_spawn_robot.c_str());
                }
            }
        
            string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch map_name:=" + map + out;
            system(launch_map.c_str());
            while (!map_active);
        
            string launch_goal_pub, launch_robot;
            for (int i = 0; i < number_of_robots; ++i){
                launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(robots[i].starting_position_x) +
                            " y:=" + to_string(robots[i].starting_position_y) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" 
                            + to_string(num_repetitions) + out;

                launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                                to_string(robots[i].goal_position_x) + " goal_y:=" + to_string(robots[i].goal_position_y) + out;

                system(launch_robot.c_str());
                while (!odom_map[robots[i].robot_namespace]);

                system(launch_goal_pub.c_str());
                while (!goal_pose_map[robots[i].robot_namespace]);
            }

            if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
                // launch tuw
                string mbf = (robots[0].nav_config == "tuw_mbf") ? "mbf:=true" : "mbf:=false";
                string launch_tuw = "roslaunch --log uml_hri_nerve_navigation tuw.launch " + mbf;
                system(launch_tuw.c_str());
                while (!tuw_active);

                string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
                system(launch_graph_checker.c_str());
            }

            string launch_goal_sender = "roslaunch --log uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                        + robot_names_str + " robot_goals:=" + robot_goals_str + " test:=" + robots[0].nav_config 
                                        + " num_robots:=" + to_string(number_of_robots) + out;
            system(launch_goal_sender.c_str());

            string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + "";
            system(launch_checker.c_str());

            sleep(1);

            string end_iteration;

            if (auto_reset) { 
                end_iteration = "rosnode list | grep -v rosaria | grep -v rosout | grep -v hokuyo | grep -v test_runner | grep -v map | xargs rosnode kill" + out;
                system(end_iteration.c_str());
                reset_robot(num_repetitions); 
            }

            if (!simulation) {
                end_iteration = "rosnode list | grep -v rosaria | grep -v rosout | grep -v hokuyo | grep -v test_runner | xargs rosnode kill" + out;
                system(end_iteration.c_str());
            }
            else {
                end_iteration = "rosnode list | grep -v gazebo | grep -v rosout | grep -v map | grep -v test_runner | xargs rosnode kill" + out;
                system(end_iteration.c_str());

                for (int i = 0; i < number_of_robots; ++i){
                    string delete_robot = "rosservice call gazebo/delete_model '{model_name: " + robots[i].robot_namespace + "}'";
                    system(delete_robot.c_str());
                }
            }
 
            sleep(1);
        }

    }

    vector<string> get_robot_names(){
        vector<string> temp;
        for (int i = 0; i < robots.size(); ++i){
            temp.push_back(robots[i].robot_namespace);
        }
        return temp;
    }

 private:
    string map, world_name, robot_names_str, robot_goals_str;
    int number_of_robots;
    vector<Robot> robots;

    // private function which resets booleans for the subscriber thread
    void reset(){
        gazebo_active = false;
        map_active = false;
        tuw_active = false;
        for (int i = 0; i < robots.size(); ++i){
            odom_map[robots[i].robot_namespace] = false;
            goal_pose_map[robots[i].robot_namespace] = false;
        }
    }

    void reset_robot(int num_repetitions) {
        string launch_goal_pub, launch_robot;
        string robot_reset_goals_str = "";
        for (int i = 0; i < number_of_robots; ++i){
            launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(xOdom) +
                           " y:=" + to_string(robots[i].goal_position_y) + " robot_name:=" + robots[i].robot_namespace 
                           + " iteration:=" + to_string(num_repetitions);
            system(launch_robot.c_str());
            while (!odom_map[robots[i].robot_namespace]);

            launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                              to_string(robots[i].starting_position_x) + " goal_y:=" + to_string(robots[i].starting_position_y);
            system(launch_goal_pub.c_str());
            while (!goal_pose_map[robots[i].robot_namespace]);

            robot_reset_goals_str += to_string(robots[i].starting_position_x) + "," + to_string(robots[i].starting_position_y);
            if (i != number_of_robots - 1){
                robot_reset_goals_str += ",";
            }
        }

        if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
            // launch tuw
            string mbf = (robots[0].nav_config == "tuw_mbf") ? "mbf:=true" : "mbf:=false";
            string launch_tuw = "roslaunch --log uml_hri_nerve_navigation tuw.launch " + mbf;
            system(launch_tuw.c_str());
            while (!tuw_active);

            string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
            system(launch_graph_checker.c_str());
        }

        string launch_goal_sender = "roslaunch --log uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                    + robot_names_str + " robot_goals:=" + robot_reset_goals_str + " test:=" + robots[0].nav_config 
                                    + " num_robots:=" + to_string(number_of_robots);
        system(launch_goal_sender.c_str());

        string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + "";
        system(launch_checker.c_str());
        sleep(1);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "automated_navigation_tester");
    ros::NodeHandle nh("~");

    string file_path, file_name;
    int num_repetitions = 1;
    file_name = "test_1.json";
    file_path = "src/uml_hri_nerve_navigation/test_defs/" + file_name;
    
    nh.getParam("repetitions", num_repetitions);
    nh.getParam("test_file", file_name);
    nh.param<std::string>("test_file_path", file_path, "src/uml_hri_nerve_navigation/test_defs/" + file_name);

    if (num_repetitions <= 0) {
        ROS_INFO("Invalid number of repetitions....... PLEASE RUN AGAIN.............\n");
        ros::shutdown();
    }

    fstream file(file_path, ios::in); 
    json json_file = json::parse(file);
    TestRunner test_runner(json_file);

    thread subscriberThread(tester_status_subscriber, test_runner.get_robot_names());
    subscriberThread.detach();

    std::cout << "RUNNING TEST FILE: " << file_name << ", TEST PATH:  " << file_path << 
            ", REPETITIONS: " << num_repetitions << std::endl;

    int flag = getc(stdin);
    cout << "\nContinuing .";

    test_runner.run(num_repetitions, true, false);

    ros::shutdown();

    return 0;
}