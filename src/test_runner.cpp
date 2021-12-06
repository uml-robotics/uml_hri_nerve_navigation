#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <chrono>
#include <unistd.h>
#include <thread>

using namespace std;
using namespace nlohmann;

int iteration = 1;

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
    string map, world_name;
    int number_of_robots;
    vector<Robot> robots;

public:
    TestRunner(json json_file){
        map = json_file.at("map").at("map_name");
        world_name = json_file.at("map").at("world_name");
        number_of_robots = json_file.at("robots").at("number_of_robots");
        robots.resize(number_of_robots);
        string robot = "robot_";
        for (int i = 0, j = 1; j < number_of_robots + 1; ++i, ++j){
            string robot_name = robot + to_string(j);
            robots.at(i).robot = json_file.at("robots").at(robot_name).at("robot");
            robots.at(i).robot_namespace = json_file.at("robots").at(robot_name).at("robot_namespace");

            robots.at(i).starting_position_x = json_file.at("robots").at(robot_name).at("starting_position").at(0);
            robots.at(i).starting_position_y = json_file.at("robots").at(robot_name).at("starting_position").at(1);

            robots.at(i).goal_position_x = json_file.at("robots").at(robot_name).at("goal_position").at(0);
            robots.at(i).goal_position_y = json_file.at("robots").at(robot_name).at("goal_position").at(1);

            robots.at(i).nav_config = json_file.at("robots").at(robot_name).at("nav_config");
        }
    }

    void run_test(){
        cout << "Running the test with " << number_of_robots << " robots" << " with the test config: " << robots.at(0).nav_config << endl;
        string out = " > output.txt &";
        // launch gazebo (in background using &)
        string launch_gazebo = "roslaunch uml_hri_nerve_nav_sim_resources spawn_world.launch world_path:=" + world_name + ".world" + out;
        system(launch_gazebo.c_str());

        sleep(2);

        // launch map (in background using &)
        string launch_map = "roslaunch uml_hri_nerve_navigation map_server.launch map_name:=" + map + out;
        system(launch_map.c_str());

        sleep(2);

        // launch robots, nav stack, loggers, if tuw (launch tuw nodes) (in background using &) setup_pionner_mbf.launch
        string launch_robot;
        string launch_goal_pub;
        for (int i = 0; i < number_of_robots; ++i){
            launch_robot = "roslaunch uml_hri_nerve_navigation setup_pioneer_mbf.launch x:=" + to_string(robots.at(i).starting_position_x) +
                           " y:=" + to_string(robots.at(i).starting_position_y) + " robot_name:=" + robots.at(i).robot_namespace + " iteration:=" + to_string(iteration) + out;
            launch_goal_pub = "roslaunch uml_hri_nerve_navigation tester_goal_publisher.launch robot_name:=" + robots.at(i).robot_namespace + " goal_x:=" +
                              to_string(robots.at(i).goal_position_x) + " goal_y:=" + to_string(robots.at(i).goal_position_y) + out;
            system(launch_robot.c_str());
            sleep(0.5);
            system(launch_goal_pub.c_str());
            sleep(2);
        }

        string launch_tuw = "roslaunch uml_hri_nerve_navigation tuw.launch mbf:=false" + out;
        system(launch_tuw.c_str());
        sleep(2);

        // graph checker 
        string launch_graph_checker = "rosrun uml_hri_nerve_navigation graph_checker";
        system(launch_graph_checker.c_str());

        // launch goals (in background using &) still need to figure out when to send goals

        string launch_goal_sender = "roslaunch uml_hri_nerve_navigation tuw_send_goals.launch" + out;
        system(launch_goal_sender.c_str());

        // launch checker
        string launch_checker = "roslaunch uml_hri_nerve_navigation robot_checker.launch > output.txt";
        system(launch_checker.c_str());

        // pkill roslaunch
        string kill_test = "pkill roslaunch";
        system(kill_test.c_str());

        // checking if roscore died
        while(ros::master::check());
        cout << "MASTER HAS DIED" << endl << endl;
        ++iteration;
    }

    void run_test_repeatedly(int iterations){
        for (int i = 0; i < iterations; ++i) {
            cout << "Running Iteration " << i + 1 << endl;
            run_test();
            cout << "Iteration " << i + 1 << " is finished" << endl;
            cout << endl;
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

    string file_path, file_name;
    int num_iterations;
    cout << "Enter the file name (file should be under test_def directory: ";

    file_name = "sample_test.json";
    file_path = "src/uml_hri_nerve_navigation/test_defs/" + file_name;
    cout << "Number of iterations: ";

    num_iterations = 2;

    fstream file(file_path, ios::in); // file path is in the directory where the executable is
    json json_file = json::parse(file);
    TestRunner test_runner(json_file);
    test_runner.run_test_repeatedly(num_iterations);

    cout << "TEST RUNNER IS ENDING" << endl;
    return 0;
}
