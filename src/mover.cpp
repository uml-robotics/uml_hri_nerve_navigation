/*  
    The mover node manages the navigation status of a robot.  First, if the robot needs to navigate towards a goal, the mover node
    gets the current navigation goal from goal_pub and commands the move_base to move toward that goal.  Once the robot reaches a goal,
    the mover node will call the goal_pub service to get a new goal if another iteration is desired.  Next, the mover node keeps track
    of the the number of iterations the robot has completed and limits the robot to a specified number of iterations.  Finally, if desired,
    the costmaps on the robot are cleared in between each iteration.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::ServiceClient costmap_service;
ros::ServiceClient goal_service;
ros::ServiceClient initial_goal_service;
ros::ServiceClient reset_goals_service;
ros::Publisher testStatusPub;

int goals_reached;
int iterations;
bool clear_costmaps;
bool initial_setup;

void goalCallback(geometry_msgs::Pose2D::ConstPtr goal_msg_pointer)
{
    geometry_msgs::Pose2D goal_msg = *goal_msg_pointer;
    move_base_msgs::MoveBaseGoal goal;

    //translate Goal Message to a MoveBaseGoal
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_msg.x;
    goal.target_pose.pose.position.y = goal_msg.y;
    tf2::Quaternion angle;
    angle.setRPY(0, 0, goal_msg.theta);
    goal.target_pose.pose.orientation.z = angle.getZ();
    goal.target_pose.pose.orientation.w = angle.getW();

    ROS_WARN("%f", goal_msg.x);

    //Send the goal to the navigation action client
    ac->sendGoal(goal);

    //Wait 1 second before checking nav status so for cases that instantly finish navigating, the code still holds for at least 1 sec
    ros::Duration(1).sleep();

    //Wait for the robot to finish moving
    ac->waitForResult();

    //Wait a little bit before requesting the next goal
    ros::Duration(2).sleep();

    std_srvs::Empty service_msg;

    if(initial_setup)
    {
        //Robot is performing the navigation test

        //Clear costmaps every iteration if clear_costmaps is true
        if(clear_costmaps)
        {
            if (!costmap_service.call(service_msg))
            {
                ROS_WARN("Failed to clear costmaps");
            }
        }

        //Increase goals_reached counter
        goals_reached++;

        //Requests another goal if the number of trips has not been met, otherwise shut down the node
        if (goals_reached / 2 < iterations)
        {
            //Get the next goal
            if (!goal_service.call(service_msg))
            {
                ROS_ERROR("Failed to request for a goal");
            }
        }
        else
        {
            //All iterations finished, publish false to test_status
            std_msgs::Bool boolean;
            boolean.data = false;
            testStatusPub.publish(boolean);

            //Node no longer needed
            ros::shutdown();
            return;
        }
    }
    else
    {
        //The robot has finished navigating to its initial position.  Either the robot sucessfully made it to its
        //initial position and the test will start or it failed and the robot will attempt to get to its initial
        //position again

        //First reset the costmap before the test starts
        if (!costmap_service.call(service_msg))
        {
            ROS_WARN("Failed to clear costmaps");
        }

        //Determine if the robot made it to its initial position
        if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            //Succeded at getting to its initial position, set initial_setup to true and get first test goal

            ROS_INFO("Robot sucessfully reached its initial position, now starting test");
            initial_setup = true;

            //Publish true to test_status
            std_msgs::Bool boolean;
            boolean.data = true;
            testStatusPub.publish(boolean);

            //Get first goal
            if (!goal_service.call(service_msg))
            {
                ROS_ERROR("Failed to request for a goal");
            }
        }
        else
        {
            //Failed at getting to its initial position, request the initial goal again

            ROS_INFO("Robot failed to reach its initial position, attempting again");

            //Get initial goal
            if (!initial_goal_service.call(service_msg))
            {
                ROS_ERROR("Failed to request for initial goal");
            }
        }
    }
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "mover_node");
    ros::NodeHandle n;

    //Subscribe to the goal topic
    ros::Subscriber sub = n.subscribe("/goal", 1, goalCallback);

    //Publisher to show whether the test is being performed or not
    testStatusPub = n.advertise<std_msgs::Bool>("/test_status", 1000, true);

    //Set up move base client and services
    ac = new MoveBaseClient("move_base", true);
    costmap_service = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    goal_service = n.serviceClient<std_srvs::Empty>("/get_new_goal");
    initial_goal_service = n.serviceClient<std_srvs::Empty>("/get_initial_goal");
    reset_goals_service = n.serviceClient<std_srvs::Empty>("/reset_goals");

    //Set defaults
    goals_reached = 0;
    iterations = 0;
    clear_costmaps = false;
    initial_setup = false;

    //Get parameters
    n.getParam(ros::this_node::getName() + "/iterations", iterations);
    n.getParam(ros::this_node::getName() + "/clear_costmaps", clear_costmaps);

    //Publish false to test_status since the test does not start immediately
    std_msgs::Bool boolean;
    boolean.data = false;
    testStatusPub.publish(boolean);

    //wait for the action server to come up
    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //wait for services
    costmap_service.waitForExistence();
    goal_service.waitForExistence();
    initial_goal_service.waitForExistence();
    reset_goals_service.waitForExistence();

    std_srvs::Empty service_msg;

    //clear the initial costmaps
    if (!costmap_service.call(service_msg))
    {
        ROS_WARN("Failed to clear costmaps");
    }

    //Call the reset goal service to ensure that the first test goal is the correct goal
    if (!reset_goals_service.call(service_msg))
    {
        ROS_ERROR("Failed to reset goals");
    }

    //Call the initial goal service to move the robots to their initial positions
    ROS_INFO("Navigating to initial test position");
    if (!initial_goal_service.call(service_msg))
    {
        ROS_ERROR("Failed to request initial goal");
    }

    //Keeps the node running and performs necessary updates
    ros::spin();

    return 0;
}
