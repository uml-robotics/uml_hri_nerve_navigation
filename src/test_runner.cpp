/*
    This file contains the TestRunner class which takes in a JSON file which should contains the definition of the test scenario which would be running multiple 
    times automatically. The JSON file is parsed when a TestRunner object is created. The TestRunner class contains two member functions for running the test
    scenarios repeatedly, run_test_repeatedly and run_test_in_person. run_test_repeatedly quits and restarts all the launch files after an iteration is ended, 
    while run_test_in_person starts the simulation and map_server only once and waits for user input (through a message being published) before beginning another
    iteration of the test (which is useful for in-person testing).
*/

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

int iteration = 1;

bool gazebo_active = false;
bool map_active = false;
bool tuw_active = false;

string tester_status = "";
ros::Publisher tester_status_pub; 

double xOdom, yOdom;
double xGoal, yGoal;

map<string, bool> odom_map;
map<string, bool> goal_pose_map;

std::fstream oStream;


float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

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
    // ROS_INFO("(%f, %f, %f)", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w);
    xOdom = msg.pose.pose.position.x;
    yOdom = msg.pose.pose.position.y;
    oStream.open("pose.txt", std::ios::out);
    oStream.setf(std::ios::fixed);
    oStream.setf(std::ios::showpoint);
    oStream.precision(2);
    oStream << "Position: (" << xOdom << ", " << yOdom << ") | Distance from goal: " << distance(xGoal, yGoal, xOdom, yOdom) << std::endl; 
    oStream.close();
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
//    cout << "--------------------------------------------------- TUW IS RUNNING --------------------------------------------------------------" << endl;
    tuw_active = true;
}

// function for another thread
void tester_status_subscriber(vector<string> robot_names){
    vector<ros::Subscriber> odom_subs, goal_pub_subs;
    odom_subs.resize(robot_names.size());
    goal_pub_subs.resize(robot_names.size());

    ros::NodeHandle nh;

    tester_status_pub = nh.advertise<std_msgs::String>("tester_status", 1000, true);
    ros::Subscriber tester_status_sub = nh.subscribe("tester_status", 1000, tester_callback);
    ros::Subscriber gazebo_sub = nh.subscribe("gazebo/model_states", 1000, gazebo_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1000, map_callback);
    ros::Subscriber tuw_sub = nh.subscribe("robot_info", 1000, tuw_callback);


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
    
    // returns a vector with robot namespaces
    vector<string> get_robot_names(){
        vector<string> temp;
        for (int i = 0; i < robots.size(); ++i){
            temp.push_back(robots[i].robot_namespace);
        }
        return temp;
    }

    void run_test(){
        cout << "Running the test with " << number_of_robots << " robots" << " with the test config: " << robots[0].nav_config << endl;
        string out = " > output.txt &";
        // launch gazebo (in background using &)
        string launch_gazebo = "roslaunch uml_hri_nerve_nav_sim_resources spawn_world.launch world_path:=" + world_name + ".world" + out;
        system(launch_gazebo.c_str());
        while(!gazebo_active);

        // launch map (in background using &)
        string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch map_name:=" + map + out;
        system(launch_map.c_str());
        while (!map_active);

        // launch robots, nav stack, loggers, if tuw (launch tuw nodes) (in background using &) setup_pionner_mbf.launch
        string launch_robot;
        string launch_goal_pub;
        for (int i = 0; i < number_of_robots; ++i){
            launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(robots[i].starting_position_x) +
                           " y:=" + to_string(robots[i].starting_position_y) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" + to_string(iteration) + out;
            launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                              to_string(robots[i].goal_position_x) + " goal_y:=" + to_string(robots[i].goal_position_y) + out;
            system(launch_robot.c_str());
            while (!odom_map[robots[i].robot_namespace]);

            system(launch_goal_pub.c_str());
            while (!goal_pose_map[robots[i].robot_namespace]);
        }

        // --------------------CHANGE/CHECK world name THIS BEFORE RUNNING------------------------
        string launch_tuw = "roslaunch uml_hri_nerve_navigation tuw.launch mbf:=false rviz_cfg:=nerve_test1" + out;
        system(launch_tuw.c_str());
        while (!tuw_active);

        // graph checker 
        string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
        system(launch_graph_checker.c_str());

        // launch goals (in background using &) still need to figure out when to send goals
        string launch_goal_sender = "roslaunch uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                    + robot_names_str + " robot_goals:=" + robot_goals_str + " test:=" + robots[0].nav_config + out; 
        system(launch_goal_sender.c_str());

        // launch checker
        string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + " > output.txt";
        system(launch_checker.c_str());

        // pkill roslaunch
        string kill_test = "pkill roslaunch";
        system(kill_test.c_str());

        // checking if roscore died
        while(ros::master::check());
        cout << "MASTER HAS DIED" << endl << endl;

        reset();

        // pauses the code until any keyboard input is received (not required?)
        int flag;
        cout << "Program is paused !\n" <<
            "Press Enter to continue\n";
        flag = getc(stdin);
        cout << "\nContinuing .";
        
        sleep(1);
    }

    // function runs the run_test functions of n (iterations) times
    void run_test_repeatedly(int iterations){
        for (int i = 0; i < iterations; ++i) {
            cout << "Running Iteration " << i + 1 << endl;
            run_test();
            cout << "Iteration " << i + 1 << " is finished" << endl;
            cout << endl;
        }
    }

    // function for running test when using physical robots (launches gazebo and map server only onces) (quicker than the other one)
    void run_test_in_person(int iterations){
        cout << "Running the test with " << number_of_robots << " robots" << " with the test config: " << robots[0].nav_config << endl;
        string out = "  &";
        string launch_gazebo = "roslaunch --log uml_hri_nerve_nav_sim_resources spawn_world.launch world_path:=" + world_name + ".world" + out;
        system(launch_gazebo.c_str());
        while(!gazebo_active);
        
        // launch map (in background using &)
        string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch map_name:=" + map + out;
        system(launch_map.c_str());
        while (!map_active);

        for (int j = 0; j < iterations; ++j){
            // publishing a pause message to avoid running another iteration without an input from user
            std_msgs::String status;
            status.data = "pause";
            tester_status_pub.publish(status);

            // launch robots, nav stack, loggers, if tuw (launch tuw nodes) (in background using &) setup_pionner_mbf.launch
            string launch_robot;
            string launch_goal_pub;
            for (int i = 0; i < number_of_robots; ++i){
                launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(robots[i].starting_position_x) +
                            " y:=" + to_string(robots[i].starting_position_y) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" + to_string(iteration) + out;

                launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                                to_string(robots[i].goal_position_x) + " goal_y:=" + to_string(robots[i].goal_position_y) + out;

                system(launch_robot.c_str());
                while (!odom_map[robots[i].robot_namespace]);
                system(launch_goal_pub.c_str());
                while (!goal_pose_map[robots[i].robot_namespace]);
            }

            if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
                string launch_tuw = "roslaunch --log uml_hri_nerve_navigation tuw.launch mbf:=true" + out;
                system(launch_tuw.c_str());
                while (!tuw_active);
            
                // graph checker 
                string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
                system(launch_graph_checker.c_str());

                // launch goals (in background using &) still need to figure out when to send goals
                string launch_goal_sender = "roslaunch --log uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                            + robot_names_str + " robot_goals:=" + robot_goals_str + " test:=" + robots[0].nav_config + " num_robots:=" + to_string(number_of_robots) + out; 
                system(launch_goal_sender.c_str());
            }
            else{
                string launch_goal_sender = "roslaunch --log uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                            + robot_names_str + " robot_goals:=" + robot_goals_str + " test:=" + robots[0].nav_config + out; 
                system(launch_goal_sender.c_str());
            }

            // launch checker
            string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + " > checker_output.txt ";
            system(launch_checker.c_str());

            int flag;
            cout << "Program is paused !\n" <<
                 "Press Enter to continue\n";
            flag = getc(stdin);
            cout << "\nContinuing .";

            cout << "--------------------------------------------- TIME TO RESET --------------------------------------------- " << endl;
            // reset_robot();
            cout << "--------------------------------------------- RESET DONE --------------------------------------------- " << endl;

            string end_iteration = "rosnode list | grep -v gazebo | grep -v rosout | grep -v map | grep -v test_runner | xargs rosnode kill" + out;
            system(end_iteration.c_str());

            for (int i = 0; i < number_of_robots; ++i){
                string delete_robot = "rosservice call gazebo/delete_model '{model_name: " + robots[i].robot_namespace + "}'";
                system(delete_robot.c_str());
            }

            // waits for a "continue" to received from /tester_status topic
            if (j != iterations-1){ 
                while(tester_status != "continue"); 
            }
        }

        // kills all launch files
        system("pkill roslaunch");
        while(ros::master::check());
        cout << "MASTER HAS DIED" << endl << endl;
    }

    void in_person_test(){
        // oStream.open(file_path_print.c_str(), std::ios::out);
        // oStream << "Position: (0.00, 0.00) | Distance from goal: 0.00" << std::endl; 
        // oStream.close();        

        string out = " &";

        // launch map server
        // string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch map_name:=" + map + out;
        string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch" + out;
        system(launch_map.c_str());
        while (!map_active);

        // launch goal pub for the checker and loggers
        string launch_robot, launch_goal_pub, launch_tuw_logger_node;
        for (int i = 0; i < number_of_robots; ++i){
            // launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(robots[i].starting_position_x) +
            //                " y:=" + to_string(robots[i].starting_position_y) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" + to_string(iteration) + out;
            launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(xOdom) +
                           " y:=" + to_string(yOdom) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" + to_string(iteration) + out;
            
            launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                              to_string(robots[i].goal_position_x) + " goal_y:=" + to_string(robots[i].goal_position_y) + out;
            cout << "GOAL POSE FOR " << robots[i].robot_namespace << " is: (" << robots[i].goal_position_x << ", " << robots[i].goal_position_y << ") " << endl;
            system(launch_robot.c_str());
            while (!odom_map[robots[i].robot_namespace]);

            system(launch_goal_pub.c_str());
            while (!goal_pose_map[robots[i].robot_namespace]);
        }
        
        int flag;
        // cout << "Program is paused !\n" <<
        //     "Press Enter to continue\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";
        
        if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
            // launch tuw
            string launch_tuw = "roslaunch uml_hri_nerve_navigation tuw.launch mbf:=true" + out;
            system(launch_tuw.c_str());
            while (!tuw_active);

            string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
            system(launch_graph_checker.c_str());

            for (int i = 0; i < number_of_robots; ++i){
                launch_tuw_logger_node = "roslaunch uml_hri_nerve_navigation tuw_logger.launch robot:=" + robots[i].robot_namespace + out;
                system(launch_tuw_logger_node.c_str());
            }
        }


        // cout << "Program is paused !\n" <<
        //      "Press Enter to continue\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";

        // launch goal sender 
        string launch_goal_sender = "roslaunch uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:="
                                    + robot_names_str + " robot_goals:=" + robot_goals_str + " test:=" + robots[0].nav_config + out; 
        system(launch_goal_sender.c_str());

        
        // launch checker
        string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + "";
        system(launch_checker.c_str());
        
        sleep(1);
        // cout << "Program is paused !\n" <<
        //      "Press Enter to continue\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";
        string end_iteration = "rosnode list | grep -v rosaria | grep -v rosout | grep -v hokuyo | grep -v test_runner | grep -v map | xargs rosnode kill" + out;
        system(end_iteration.c_str());
        sleep(1);
        reset_robot();
        
        end_iteration = "rosnode list | grep -v rosaria | grep -v rosout | grep -v hokuyo | grep -v test_runner | xargs rosnode kill" + out;
        system(end_iteration.c_str());
    }

    void reset_robot(){
        string out = " &";

        // if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
        //     system("rosnode kill multi_robot_router");
        //     system("rosnode kill rviz");
        //     system("rosnode kill graph_generator");
            
        //     string kill_goal_sender = "rosnode kill tuw_goal_sender_node";
        //     system(kill_goal_sender.c_str());
        //     cout << "----------------- KILLED THE GOAL SENDER" << endl;
        // }

        int flag;

        // string topic1, topic2, topic3;
        // for (int i = 0; i < number_of_robots; ++i){
        //     topic1 = "rosnode kill " + robots[i].robot_namespace + "/link1_broadcaster";
        //     topic2 = "rosnode kill " + robots[i].robot_namespace + "/local_behavior";
        //     topic3 = "rosnode kill " + robots[i].robot_namespace + "/self_localization";
        //     system(topic1.c_str());
        //     system(topic2.c_str());
        //     system(topic3.c_str());
        // }


        // cout << "--------------------------------------------------------------------------------------Program is paused !\n" <<
        //     "Press Enter to continue-------------------------------------------------------------------------------------\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";

        xGoal = robots[0].starting_position_x;
        yGoal = robots[0].starting_position_x;

        // launch goal pub
        string launch_goal_pub, launch_robot;
        string robot_reset_goals_str = "";
        for (int i = 0; i < number_of_robots; ++i){
            launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(xOdom) +
                           " y:=" + to_string(robots[i].goal_position_y) + " robot_name:=" + robots[i].robot_namespace + " iteration:=" + to_string(iteration) + out;
            system(launch_robot.c_str());
            while (!odom_map[robots[i].robot_namespace]);

            launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots[i].robot_namespace + " goal_x:=" +
                              to_string(robots[i].starting_position_x) + " goal_y:=" + to_string(robots[i].starting_position_y) + out;
            system(launch_goal_pub.c_str());
            while (!goal_pose_map[robots[i].robot_namespace]);
            robot_reset_goals_str += to_string(robots[i].starting_position_x) + "," + to_string(robots[i].starting_position_y);
            if (i != number_of_robots - 1){
                robot_reset_goals_str += ",";
            }
        }


        // cout << "--------------------------------------------------------------------------------------Program is paused !\n" <<
        //     "Press Enter to continue-------------------------------------------------------------------------------------\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";
        
        if (robots[0].nav_config == "tuw" || robots[0].nav_config == "tuw_mbf"){
            // launch tuw
            string launch_tuw = "roslaunch --log uml_hri_nerve_navigation tuw.launch mbf:=true &";
            system(launch_tuw.c_str());
            while (!tuw_active);

            string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
            system(launch_graph_checker.c_str());
        }


    //    cout << "--------------------------------------------------------------------------------------Program is paused !\n" <<
    //         "Press Enter to continue-------------------------------------------------------------------------------------\n";
    //    flag = getc(stdin);
    //    cout << "\nContinuing .";


        // send goals
        string launch_goal_sender = "roslaunch --log uml_hri_nerve_navigation multiple_robots_test_goal_sender.launch robot_names:=" 
                                    + robot_names_str + " robot_goals:=" + robot_reset_goals_str + " test:=" + robots[0].nav_config + " num_robots:=" + to_string(number_of_robots) + out;
        system(launch_goal_sender.c_str());

        cout << "----------------------------------- LAUNCHED THE MOVER" << endl;
        // checker
        string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch robot_names:=" + robot_names_str + "";
        system(launch_checker.c_str());
        sleep(1);

        // cout << "Program is paused !\n" <<
        //     "Press Enter to continue\n";
        // flag = getc(stdin);
        // cout << "\nContinuing .";

    }

    void run_in_person_test_repeatedly(int iterations){
        for (int i = 0; i < iterations; ++i){
            cout << "----------------------------------------------- RUNNING ITERATION: " << i + 1 << 
            " -----------------------------------------------" << endl;
            
            in_person_test();

            cout << "----------------------------------------------- ENDING ITERATION: " << i + 1 << 
            " -----------------------------------------------" << endl;

            // system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=pioneer/cmd_vel");
            
            // while(tester_status != "continue");
            // tester_status = "pause";

            sleep(2);
            // int flag;
            // cout << "Program is paused !\n" <<
            //     "Press Enter to continue\n";
            // flag = getc(stdin);
            // cout << "\nContinuing .";

            xGoal = robots[0].goal_position_x;
            yGoal = robots[0].goal_position_y;

            // system("rosnode kill /teleop_twist_keyboard");            
        }
    }
};  

void time_counter(chrono::steady_clock::time_point start){
    fstream outFile;
    while(true) {
        sleep(1);
        auto end = std::chrono::steady_clock::now();
        double time_elapsed = double(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        int minutes; int seconds;
        double time_elapsed_sec = time_elapsed/1e9;
        if (time_elapsed_sec <= 60){
            minutes = 0;
            seconds = time_elapsed_sec;
        }
        else{
            minutes = time_elapsed_sec/60;
            seconds = (int)time_elapsed_sec % 60;
        }
        outFile.open("time.txt", ios::out);
        outFile << setw(2) << setfill('0') << minutes << ":" << setw(2) << setfill('0') << seconds << std::endl;
        outFile.close();
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "test_runner");
    auto start = std::chrono::steady_clock::now();

    thread time_counter_thread(time_counter, start);
    time_counter_thread.detach();

    string file_path = " ", file_name = " ";
    int num_iterations = 2;

    file_name = "test_1.json";
    file_path = "src/uml_hri_nerve_navigation/test_defs/" + file_name;

    fstream file(file_path, ios::in); // file path is in the directory where the executable is
    json json_file = json::parse(file);
    TestRunner test_runner(json_file);

    // starting the other thread
    thread subscriberThread(tester_status_subscriber, test_runner.get_robot_names());
    subscriberThread.detach();

    //  test_runner.run_in_person_test_repeatedly(2);
    cout << "THE CODE IS RUNNING TILL HERE: 1" << endl;
    test_runner.run_in_person_test_repeatedly(31);

    system("clear");

    ros::shutdown();

    return 0;
}