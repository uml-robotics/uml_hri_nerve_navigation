#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

//SimLog is a custom message type defined in uml_hri_nerve_navigation/msg/SimLog.msg
#include <uml_hri_nerve_navigation/SimLog.h>

// Simple distance function
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

class Logger
{
private:
    //Publisher
    ros::Publisher pubLog;
    ros::Publisher pubConfig;

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

public:
    Logger(ros::NodeHandle n)
    {
        // Setup publisher
        pubLog = n.advertise<uml_hri_nerve_navigation::SimLog>("sim_log", 1000, false);
        pubConfig = n.advertise<std_msgs::String>("robot_config", 1000, true);

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
    };

    void add_position(geometry_msgs::PoseWithCovarianceStamped pose)
    {
        if (navigating && test_active)
        {
            //save the position and distance
            log.robot_pos.x = pose.pose.pose.position.x;
            log.robot_pos.y = pose.pose.pose.position.y;
            tf2::Quaternion quaternion;
            quaternion.setX(pose.pose.pose.orientation.x);
            quaternion.setY(pose.pose.pose.orientation.y);
            quaternion.setZ(pose.pose.pose.orientation.z);
            quaternion.setW(pose.pose.pose.orientation.w);
            tf2::Matrix3x3 matrix;
            matrix.setRotation(quaternion);
            double roll, pitch, yaw;
            matrix.getEulerYPR(yaw, pitch, roll);
            log.robot_pos.theta = yaw * 180.0 / M_PI;
            log.dist_from_goal = distance(log.goal.x, log.goal.y, pose.pose.pose.position.x, pose.pose.pose.position.y);
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

    void add_goal(geometry_msgs::Pose2D goal)
    {
        if (!navigating && test_active)
        {
            //Save goal pos
            log.goal.x = goal.x;
            log.goal.y = goal.y;

            //Change nav and publish state
            navigating = true;
            publishing = true;

            //Increment iterations
            log.iteration++;

            //Reset iteration collision counter
            iteration_collision = 0;

            //Add msg log
            std::stringstream str;
            str << "Goal registered at x:" << goal.x << " y:" << goal.y;
            log.event = str.str();
            ROS_INFO("Starting iteration %d", log.iteration);
            ROS_INFO("Goal registered at x:%.3f y:%.3f", goal.x, goal.y);
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
            log.header.stamp = ros::Time::now();
            pubLog.publish(log);

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

        //Start config with a quote for multiline formatting in csv
        config = "\"";
        
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

        //Close config with a quote for multiline formatting in csv
        config += "\"";

        //Publish the config
        std_msgs::String configMsg;
        configMsg.data = config;
        pubConfig.publish(configMsg);
    }
};

Logger *logger;

void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
    logger->add_position(*pose);
}

void collision_callback(const gazebo_msgs::ContactsState::ConstPtr &collision)
{
    logger->add_collision(*collision);
}

void goal_callback(const geometry_msgs::Pose2D::ConstPtr &goal)
{
    logger->add_goal(*goal);
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

void test_status_callback(const std_msgs::Bool::ConstPtr &test_state)
{
    logger->set_test_status(*test_state);
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "logger");
    ros::NodeHandle n;

    ros::Rate rate(10);
    ros::AsyncSpinner spinner(5);

    // Setup subscribers
    ros::Subscriber odom_sub = n.subscribe("amcl/amcl_pose", 1, odom_callback);
    ros::Subscriber collision_sub = n.subscribe("bumper_contact", 10, collision_callback);
    ros::Subscriber goal_sub = n.subscribe("/goal", 10, goal_callback);
    ros::Subscriber move_base_result = n.subscribe("move_base/result", 10, state_callback);
    ros::Subscriber test_status = n.subscribe("/test_status", 10, test_status_callback);

    logger = new Logger(n);

    //log the config
    logger->log_config();

    spinner.start();

    while (ros::ok())
    {
        logger->publish_log();
        ros::spinOnce();
        rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}
