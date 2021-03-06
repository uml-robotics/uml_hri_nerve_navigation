/* 
    The detected obstacle reaction distance (DORD) calculator calculates the DORD metric on the nerve_long_hall map. This metric measures the distance 
    between the pallet and the robot that the robot detects the obstacle and turns around. This node subscribes to a position topic for the robot and looks 
    for when the robot is traveling down the left aisle with the obstacle.  Then this node will save the position that gets the closest to the obstacle. 
    Finally the distance between the edge of the obstacle and the center of the robot is calculated and saved to csv log.  For this calculator to work,
    it assumes that desired obstacle to calculate the distance on is the first observed obstacle when traveling from goal a to b and that the distance between 
    the robot and the obstacle is always decreasing when approaching the obstacle.
*/

#include <ros/ros.h>
#include <math.h>
#include <uml_hri_nerve_navigation/Goal.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/Bool.h>
#include <string>
#include <sstream>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <boost/filesystem.hpp>

// Simple distance function
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

class DordCalculator
{
private:
    //Node Handle
    ros::NodeHandle nh;

    //Msgs
    geometry_msgs::PointStamped currentPosition;
    geometry_msgs::Pose2D initialPosition;

    volatile float dordDistance;

    //dord distance timestamp
    ros::Time timestamp;

    //Names of goals that the robot is navigating to 
    std::string goal;

    //Obstacle location
    float obstacle_x;
    float obstacle_y;

    bool navigatingToGoal;
    bool testActive;
    bool newGoalRegistered;

    int currentDistanceBiggerThanDord;

    //Files
    std::fstream csvFile;

public:
    DordCalculator(ros::NodeHandle n)
    {
        //Setup variable defaults
        nh = n;

        dordDistance = 1000; //make it large since a minimum value is being found

        navigatingToGoal = false;
        testActive = false;

        currentDistanceBiggerThanDord = 0;

        //Get the two goal names and the location for the edge of the obstacle
        n.getParam(ros::this_node::getName()+"/goal_name", goal);
        n.getParam(ros::this_node::getName()+"/obstacle_x", obstacle_x);
        n.getParam(ros::this_node::getName()+"/obstacle_y", obstacle_y);

        //Get file path from parameter
        std::string file_path;
        n.getParam(ros::this_node::getName()+"/file_path", file_path);
        std::string file_path_csv = file_path + "/dord_metric.csv";

        //Create the folders
        boost::filesystem::create_directories(file_path.c_str());
        boost::filesystem::permissions(file_path.c_str(), boost::filesystem::all_all);

        //Create files
        csvFile.open(file_path_csv.c_str(), std::ios::out);

        //Write the headers for the csv file
        csvFile<<"Timestamp,DORD Metric" << std::endl;
    };

    //This function updates and checks if a dord distance needs to be logged.  This function should be run inside of the main loop
    void run()
    {
        //Run the dord calculation and determine if the calculated distance is correct
        bool metricFound = has_dord_metric_been_found();
        if(metricFound)
        {
            //Log the dord distance
            log_metric();

            //Set navigatingToGoal to false so the metric won't be calculated for the rest of the iteration
            navigatingToGoal = false;

            //Reset Dord Distance
            dordDistance = 1000;

        }
    }

    //This function calculates the dord metric and determines if the current dord distance it has found is the correct distance
    bool has_dord_metric_been_found()
    {
        //Test has to be active and the robot must be navigating to goal b to consider calculating the metric
        if(testActive && navigatingToGoal)
        {
            //Check if the robot is still close to its initial position.  If so, the robot can start off by moving away from the obstacle and cause 
            //a false positive
            float initial_x_space = abs(initialPosition.x - currentPosition.point.x);
            float initial_y_space = abs(initialPosition.y - currentPosition.point.y);
            if(initial_x_space < 2 && initial_y_space < 2)
            {
                return false;
            }

            //Next calculate the current dord distance and check if it is smaller then the saved dord distance.
            float currentDistance = distance(currentPosition.point.x, currentPosition.point.y, obstacle_x, obstacle_y);
            if(currentDistance < dordDistance)
            {
                //set the new dord distance
                dordDistance = currentDistance;
                timestamp = currentPosition.header.stamp;
                //reset the counter for occurances that dord distance has been larger than current distance
                currentDistanceBiggerThanDord = 0;
            }
            else
            {
                //increase occurances that current distance has been larger
                currentDistanceBiggerThanDord++;
            }

            //Check if the current distance has been consistenly larger than dord distance
            if(currentDistanceBiggerThanDord > 9)
            {
                //Robot has probably turn away from obstacle and thus the dord distance has been found
                ROS_INFO("DORD metric found to be %fm", dordDistance);
                return true;
            }
        }

        //If code reaches this point, then the dord distance has not been found
        return false;
    }

    //This function keeps track of the robots current position
    void update_current_position(geometry_msgs::PoseStamped pose)
    {
        //Save the current position and timestamp.  The x and y coordinates are the only two coordinates that matter in this 
        //calculation and the other fields can be discarded.
        currentPosition.point.x = pose.pose.position.x;
        currentPosition.point.y = pose.pose.position.y;
        currentPosition.header.stamp = pose.header.stamp;

        //if the new goal was just registered, save the initial position
        if(newGoalRegistered)
        {
            //save the initial position
            initialPosition.x = pose.pose.position.x;
            initialPosition.y = pose.pose.position.y;

            //reset new goal boolean
            newGoalRegistered = false;
        }
    }

    //This function sets the current status of the test.  If a test is not active, don't bother calculating dord metric
    void set_test_status(std_msgs::Bool test_status_msg)
    {
        testActive = test_status_msg.data;
    }

    //This function recieves the current goal and determines if the robot is navigating toward goal b
    void navigating_toward_desired_goal(uml_hri_nerve_navigation::Goal current_goal)
    {
        //Find if the current goal's description matches the description for the desired goal
        if(std::strcmp(goal.c_str(), current_goal.description.c_str()) == 0)
        {
            navigatingToGoal = true;
        }
        else
        {
            navigatingToGoal = false;
        }

        //Update new goal boolean
        newGoalRegistered = true;
    }

    //This functions checks if the robot has stopped navigating and automatically sets the navigating to goal b status to false if not navigating
    void set_current_navigation_status(bool is_navigating)
    {
        if(!is_navigating)
            navigatingToGoal = false;
    }

    void log_metric()
    {
        //Log the current dord distance
        csvFile << std::to_string(timestamp.toNSec()) << "," << std::to_string(dordDistance) << std::endl;
    }

    void close_file()
    {
        //Closes the two files that are being written to at the end of the node's life
        csvFile.close();
    }
};

DordCalculator *calculator;

void odom_callback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    calculator->update_current_position(*pose);
}

void goal_callback(const uml_hri_nerve_navigation::Goal::ConstPtr &goal)
{
    calculator->navigating_toward_desired_goal(*goal);
}

void state_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &state)
{
    if (state->status.status == state->status.ABORTED || state->status.status == state->status.REJECTED || state->status.status == state->status.SUCCEEDED)
    {
        calculator->set_current_navigation_status(false);
    }
    
}

void test_status_callback(const std_msgs::Bool::ConstPtr &test_state)
{
    calculator->set_test_status(*test_state);
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "dord_calculator");
    ros::NodeHandle n;

    ros::Rate rate(10);
    ros::AsyncSpinner spinner(4);

    // Setup subscribers
    ros::Subscriber odom_sub = n.subscribe("map_pose", 1, odom_callback);
    ros::Subscriber goal_sub = n.subscribe("goal", 10, goal_callback);
    ros::Subscriber move_base_result = n.subscribe("move_base/result", 10, state_callback);
    ros::Subscriber test_status = n.subscribe("/test_status", 10, test_status_callback);

    calculator = new DordCalculator(n);

    spinner.start();

    while (ros::ok())
    {
        //Start running the calculator
        calculator->run();
        ros::spinOnce();
        rate.sleep();
    }

    //Close the csv file in the calculator
    calculator->close_file();

    delete calculator;

    ros::waitForShutdown();

    return 0;
}
