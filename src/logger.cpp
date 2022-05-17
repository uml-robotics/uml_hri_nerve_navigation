/*
    The logger node records all relevent information needed during a navigation test and logs the information into a csv and a txt file.  All of the one-time initial paramters are saved to the text file
    and all of the parameters that change during the duration of a test are saved in the csv file.  Some examples of the data that is recorded are the robot's position, the current goal location, and the
    current distance to the goal.  The logs are also sorted out into folders that are named using the current level, robot, and the initial timestamp of the log.  The logs can be found on the resources/logs
    folder.
*/

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <uml_hri_nerve_navigation/Goal.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <sstream>
#include <list>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <mbf_msgs/ExePathActionResult.h>
#include <tuw_local_controller_msgs/ExecutePathActionResult.h>
//SimLog is a custom message type defined in uml_hri_nerve_navigation/msg/SimLog.msg
#include <uml_hri_nerve_navigation/SimLog.h>
#include <chrono>
#include <thread>
#include <nav_msgs/Odometry.h>


// Simple distance function
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

int minutes; 
int seconds;
bool thread_started = false;
void time_counter(std::chrono::steady_clock::time_point start){
    using namespace std;
    while(true) {
        sleep(1);
        auto end = std::chrono::steady_clock::now();
        double time_elapsed = double(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        double time_elapsed_sec = time_elapsed/1e9;
        if (time_elapsed_sec <= 60){
            minutes = 0;
            seconds = time_elapsed_sec;
        }
        else{
            minutes = time_elapsed_sec/60;
            seconds = (int)time_elapsed_sec % 60;
        }
        // outFile.open("time.txt", ios::out);
        // outFile << setw(2) << setfill('0') << minutes << ":" << setw(2) << setfill('0') << seconds << std::endl;
        // outFile.close();
    }
}

class Logger
{
private:
    //Node Handle
    ros::NodeHandle nh;

    //Msgs
    uml_hri_nerve_navigation::SimLog log;
    std_msgs::String config;

    //Other variables
    uint collision_num;
    uint iteration_collision;
    double collision_x;
    double collision_y;
    double collision_thres;
    uint goals_reached;
    uint goals_aborted;
    bool navigating;
    bool publishing;
    bool test_active;

    //Files
    std::fstream csvFile;
    std::fstream txtFile;
    std::fstream oStream;

    std::string filePathString;
    std::string filePathVel;

    int num;

public:
    Logger(ros::NodeHandle n)
    {

        std::cout << "------------------------------------------------- THIS VERSION OF LOGGER IS RUNNING ---------------------------------------------" << std::endl;
        //Setup variable defaults
        nh = n;
        collision_num = 0;
        iteration_collision = 0;
        collision_x = 0;
        collision_y = 0;
        collision_thres = 0.3;
        goals_reached = 0;
        goals_aborted = 0;
        navigating = false;
        publishing = false;
        test_active = false;

        log.collision.x = -100000;
        log.collision.y = -100000;

        log.iteration = 0;

        //Get fcsvile path from parameter
        std::string file_path;
        n.getParam(ros::this_node::getName()+"/file_path", file_path);
        filePathString = file_path;
        filePathVel = file_path + "/velocity.txt";
        std::string file_path_csv = file_path + "/sim_log.csv";
        std::string file_path_txt = file_path + "/config.txt";
        std::string file_path_print = file_path + "/print.txt";


        //Create the folders
        //boost::filesystem::create_directories(file_path.c_str());
        //boost::filesystem::permissions(file_path.c_str(), boost::filesystem::all_all);

        //Create files
        csvFile.open(file_path_csv.c_str(), std::ios::out | std::ios_base::app);
        txtFile.open(file_path_txt.c_str(), std::ios::out);
        // oStream.open(file_path_print.c_str(), std::ios::out);

        std::string file_path_iteration = file_path + "/num_iteration.txt";
        std::ifstream inStream;
        std::fstream outStream;

        if (!csvFile){
            int temp = 0;
            boost::filesystem::create_directories(file_path.c_str());
            boost::filesystem::permissions(file_path.c_str(), boost::filesystem::all_all);
            csvFile.open(file_path_csv.c_str(), std::ios::out);
            txtFile.open(file_path_txt.c_str(), std::ios::out);
            // oStream.open(file_path_print.c_str(), std::ios::out);

            outStream.open(file_path_iteration.c_str(), std::ios::out);
            outStream << temp;
            outStream.close();
        }
        std::string strTemp;

        inStream.open(file_path_iteration.c_str());
        inStream >> num;
        if (num != 0) {
            inStream >> strTemp;
        }
        inStream.close();


        ROS_ERROR("--------------------------------------------------- LOGGER: |%d|%s|----------------------------------------------------", num, strTemp.c_str());

         if (num == 0 || strTemp == "(Reset)") {
            ROS_ERROR("----------------------------------------------------------------- HERE");
            num++;
            outStream.open(file_path_iteration.c_str(), std::ios::out);
            outStream << num;
            outStream << " (Navigation)";
            outStream.close();
        }
        else if (strTemp == "(Navigation)") {
            outStream.open(file_path_iteration.c_str(), std::ios::out);
            outStream << num;
            outStream << " (Reset)";
            outStream.close();
        }

        oStream.open(file_path_print.c_str(), std::ios::out);
        oStream << "Position: (0.00, 0.00) | Distance from goal: 0.00" << std::endl; 
        oStream.close();        

        csvFile<<"Iteration,Elapsed Time,Timestamp,Robot Pos X,Robot Pos Y,Robot Pos Theta,Goal X,Goal Y,Distance From Goal,Collision X,Collision Y,Goal#,Event" << std::endl;

    };

    void add_velocity(geometry_msgs::Twist cmd_vel){
        ROS_ERROR("--------------------------------------------------- LOGGER VELOCITY ----------------------------------------------------");

        std::fstream velFile;
        velFile.setf(std::ios::fixed);
        velFile.setf(std::ios::showpoint);
        velFile.precision(2);
        if (navigating && test_active) {
            velFile.open(filePathVel.c_str(), std::ios::out);
            velFile << "Velocity: Linear: " << cmd_vel.linear.x << " | Angular: " << cmd_vel.angular.z << std::endl;
            velFile.close();
        }
    }
    void add_position(geometry_msgs::PoseStamped pose)
    {
        if (navigating && test_active)
        {
            //save the position and distance
            log.robot_pos.x = pose.pose.position.x;
            log.robot_pos.y = pose.pose.position.y;
            tf2::Quaternion quaternion;
            quaternion.setX(pose.pose.orientation.x);
            quaternion.setY(pose.pose.orientation.y);
            quaternion.setZ(pose.pose.orientation.z);
            quaternion.setW(pose.pose.orientation.w);
            tf2::Matrix3x3 matrix;
            matrix.setRotation(quaternion);
            double roll, pitch, yaw;
            matrix.getEulerYPR(yaw, pitch, roll);
            log.robot_pos.theta = yaw * 180.0 / M_PI;
        }
    }

    void add_collision(gazebo_msgs::ContactsState collision)
    {
        if (navigating && !collision.states.empty() && test_active)
        {
            //Filters out collisions that are right next to each other
            if (iteration_collision != 0 && std::abs(collision.states[0].contact_positions[0].x - collision_x < collision_thres) && std::abs(collision.states[0].contact_positions[0].y - collision_y < collision_thres))
            {
                return;
            }

            //At this point the collision registered is valid and needs to be logged

            //Intrement runs with collision counter if this is the first collision reported in the current iteration
            if (iteration_collision == 0)
            {
                collision_num++;
            }

            //Increment the num of collisions in the current iteration and save coords
            iteration_collision++;
            collision_x = collision.states[0].contact_positions[0].x;
            collision_y = collision.states[0].contact_positions[0].y;

            //Add collision to msg
            log.collision.x = collision_x;
            log.collision.y = collision_y;

            //Add msg log
            ROS_WARN("Collision detected at x:%.3f y:%.3f", log.collision.x, log.collision.y);
            std::stringstream str;
            str << "Collision detected at x:" << log.collision.x << " y:" << log.collision.y;
            log.event = str.str();
        }
    }

    void add_nav_result(bool goal_reached)
    {
        if (navigating && test_active)
        {
            //Change navigation state
            navigating = false;
            if (goal_reached)
            {
                //Add msg log
                ROS_INFO("The goal was reached");
                log.event = "Goal was reached";
            }
            else
            {
                //Add msg log
                ROS_WARN("The goal was not reached");
                log.event = "Goal was not reached";
            }

            //Output an iteration is ending if trips is even
            ROS_INFO("Ending iteration %d", log.iteration);
        }
    }

    void add_goal(uml_hri_nerve_navigation::Goal goal)
    {
        if (!navigating && test_active)
        {
            //Save goal pos
            log.goal.x = goal.goal.x;
            log.goal.y = goal.goal.y;

            //Change nav and publish state
            navigating = true;
            publishing = true;

            //Increment iterations
            log.iteration++;

            //Reset iteration collision counter
            iteration_collision = 0;

            //Add msg log
            std::stringstream str;
            str << "Goal: " << goal.description << " registered at x:" << goal.goal.x << " y:" << goal.goal.y;
            log.event = str.str();
            ROS_INFO("Starting iteration %d", log.iteration);
            ROS_INFO("Goal: %s registered at x:%.3f y:%.3f", goal.description.c_str(), goal.goal.x, goal.goal.y);
        }
    }

    void mbf_add_goal(geometry_msgs::Pose2D goal_pose){
        if (!navigating && test_active)
        {
            //Save goal pos
            log.goal.x =  goal_pose.x;
            log.goal.y =  goal_pose.y;

            //Change nav and publish state
            navigating = true;
            publishing = true;

            //Increment iterations
            log.iteration++;

            //Reset iteration collision counter
            iteration_collision = 0;
            
            int num = log.iteration;

            //Add msg log
            std::stringstream str;
            str << "Goal: " << num << " registered at x:" << goal_pose.x << " y:" << goal_pose.y;
            log.event = str.str();
            ROS_INFO("Starting iteration %d", log.iteration);
            ROS_INFO("Goal: %d registered at x:%.3f y:%.3f", num, goal_pose.x, goal_pose.y);
        }
    }

    void set_test_status(std_msgs::Bool test_status_msg)
    {
        test_active = test_status_msg.data;
    }

    void publish_log()
    {
        if (publishing)
        {
            if (!thread_started){
                auto start = std::chrono::steady_clock::now();  
                std::thread time_counter_thread(time_counter, start);
                time_counter_thread.detach();
                thread_started = true;
            }
            //Save the current timestamp for the log
            log.header.stamp = ros::Time::now();
            std::string elapsed_time_str;
            if (std::to_string(minutes).size() <= 1){
                elapsed_time_str = "0" + std::to_string(minutes);
            }
            else{
                elapsed_time_str = std::to_string(minutes);
            }
            elapsed_time_str += ":";
            if (std::to_string(seconds).size() <= 1){
                elapsed_time_str += "0"+std::to_string(seconds);
            }
            else{
                elapsed_time_str += std::to_string(seconds);
            }

            // std::string elapsed_time_str = std::to_string(minutes) + ":" + std::to_string(seconds);
            std::cout << "ELAPSED: " << elapsed_time_str << std::endl;

            //Calculate the distance from the current pos to the goal.  This line is here instead of add_position because the goal can change, 
            //but the position doesn't and the distance calculation won't be performed until the position changes
            log.dist_from_goal = distance(log.goal.x, log.goal.y, log.robot_pos.x, log.robot_pos.y);
            
            //Write the contents of the log msg to the csv file (replace with publisher .publish if you want to publish a sim_log topic)
            csvFile <<  num << "," << elapsed_time_str << "," << std::to_string(log.header.stamp.toSec()) << "," << log.robot_pos.x << "," << log.robot_pos.y << "," << log.robot_pos.theta << "," <<
                    log.goal.x << "," << log.goal.y << "," << log.dist_from_goal << "," << log.collision.x << "," << log.collision.y << "," <<
                    std::to_string(log.iteration) << "," << log.event << std::endl;
            

            std::string file_path_print = filePathString + "/print.txt";
            oStream.open(file_path_print.c_str(), std::ios::out);
            oStream.setf(std::ios::fixed);
            oStream.setf(std::ios::showpoint);
            oStream.precision(2);
            oStream << "Position: (" << log.robot_pos.x << ", " << log.robot_pos.y << ") | Distance from goal: " << log.dist_from_goal << std::endl; 
            oStream.close();

            
            //Reset collision coords if a collision was published
            if (log.collision.x != -100000 || log.collision.y != -100000)
            {
                log.collision.x = -100000;
                log.collision.y = -100000;
            }

            //Reset event string if an event was published
            if (log.event != "")
            {
                log.event.clear();
            }
        }
        if (!navigating)
        {
            publishing = false;
        }
    }

    void log_config()
    {
        //Create string to start build the config log
        std::string config;
        
        //Save the type of local planner being used
        std::string local_planner;
        ros::param::get("/move_base/base_local_planner", local_planner);
        int slashIndex = local_planner.find('/');
        local_planner = local_planner.substr(slashIndex + 1);
        
        //String to save parameters into
        double parameter;

        //Log the max speed the robot is set to travel
        ros::param::get("/move_base/" + local_planner + "/max_vel_x", parameter);
        config += "Max Speed: " + std::to_string(parameter) + "\n";

        //log the set distance obstacles start to be recognized
        ros::param::get("/move_base/local_costmap/obstacle_range", parameter);
        config += "Obstacle Marking Distance: " + std::to_string(parameter);

        //Publish the config
        txtFile << config;
    }

    void close_files()
    {
        //Closes the two files that are being written to at the end of the node's life
        csvFile.close();
        txtFile.close();
        oStream.close();
    }
};

Logger *logger;

void odom_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    // geometry_msgs::PoseStamped pose;
    // pose.pose = pose_msg->pose.pose;
    logger->add_position(*pose_msg);
}
void velocity_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    logger->add_velocity(*cmd_vel);
}

void collision_callback(const gazebo_msgs::ContactsState::ConstPtr &collision)
{
    logger->add_collision(*collision);
}

void goal_callback(const uml_hri_nerve_navigation::Goal::ConstPtr &goal)
{
    logger->add_goal(*goal);
}

void mbf_goal_callback(const geometry_msgs::Pose2D::ConstPtr &goal)
{
    logger->mbf_add_goal(*goal);
}

void state_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &state)
{
    if (state->status.status == state->status.ABORTED || state->status.status == state->status.REJECTED)
    {
        logger->add_nav_result(false);
    }

    if (state->status.status == state->status.SUCCEEDED)
    {
        logger->add_nav_result(true);
    }
    
}

void mbf_state_callback(const std_msgs::Bool::ConstPtr &state)
{
    logger->add_nav_result(state->data);    
}

// tuw_local_controller_msgs/ExecutePathActionResult
void tuw_state_callback(const tuw_local_controller_msgs::ExecutePathActionResult::ConstPtr &state)
{
    if (state->status.status == state->status.ABORTED || state->status.status == state->status.REJECTED)
    {
        logger->add_nav_result(false);
    }

    if (state->status.status == state->status.SUCCEEDED)
    {
        logger->add_nav_result(true);
    }
    
}

void test_status_callback(const std_msgs::Bool::ConstPtr &test_state)
{
    logger->set_test_status(*test_state);
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "logger");
    ros::NodeHandle n;

    ros::Rate rate(20);
    ros::AsyncSpinner spinner(5);
    int place_holder;
    bool mbf = false;
    bool tuw = false;
    n.getParam(ros::this_node::getName()+"/mbf", mbf);
    n.getParam(ros::this_node::getName()+"/tuw", tuw);

    n.getParam(ros::this_node::getName()+"/iteration_number", place_holder);


    ros::Subscriber odom_sub = n.subscribe("map_pose", 1, odom_callback);
    ros::Subscriber collision_sub = n.subscribe("bumper_contact", 10, collision_callback);
    ros::Subscriber test_status = n.subscribe("test_status", 10, test_status_callback);
    ros::Subscriber goal_sub = n.subscribe("goal", 10, mbf_goal_callback);
    ros::Subscriber vel_sub = n.subscribe("cmd_vel", 10, velocity_callback);                // ======================= HARD CODED
    ros::Subscriber move_base_result;

    if (mbf) {
        move_base_result = n.subscribe("result", 10, mbf_state_callback);
    }
    else if (tuw){
        move_base_result = n.subscribe("result", 10, tuw_state_callback);
    }
    else {
        ROS_ERROR("MBF FALSE");
        move_base_result = n.subscribe("result", 10, state_callback);
    }
    

    logger = new Logger(n);

    //log the config
    logger->log_config();

    spinner.start();

    while (ros::ok())
    {
        //Start saving the log
        logger->publish_log();
        ros::spinOnce();
        rate.sleep();
    }

    //Close the csv file in the logger
    logger->close_files();

    delete logger;

    ros::waitForShutdown();

    return 0;
}