/*  
    The mover node manages the navigation status of a robot.  First, if the robot needs to navigate towards a goal, the mover node
    gets the current navigation goal from goal_pub and commands the move_base to move toward that goal.  Once the robot reaches a goal,
    the mover node will call the goal_pub service to get a new goal if another iteration is desired.  Next, the mover node keeps track
    of the the number of iterations the robot has completed and limits the robot to a specified number of iterations.  Finally, if desired,
    the costmaps on the robot are cleared in between each iteration.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::ServiceClient costmap_service;
ros::ServiceClient goal_service;

int goals_reached;
int iterations;
bool clear_costmaps;

void goalCallback(geometry_msgs::Pose2D goal_msg)
{
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

    //Send the goal to the navigation action client
    ac->sendGoal(goal);

    //Wait for the robot to finish moving
    ac->waitForResult();

    std_srvs::Empty service_msg;

    //Clear costmaps once finished navigating if clear_costmaps is true
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
        //Wait a little bit before requesting the next goal
        ros::Duration(2).sleep();

        //Get the next goal
        if (!goal_service.call(service_msg))
        {
            ROS_ERROR("Failed to request for a goal");
        }
    }
    else
    {
        //All iterations finished, node no longer needed
        ros::shutdown();
        return;
    }
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "mover_node");
    ros::NodeHandle n;

    //Subscribe to the goal topic
    ros::Subscriber sub = n.subscribe("/goal", 10, goalCallback);
    
    ac = new MoveBaseClient("move_base", true);
    costmap_service = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    goal_service = n.serviceClient<std_srvs::Empty>("/get_new_goal");

    //Set defaults
    goals_reached = 0;
    iterations = 0;
    clear_costmaps = false;

    //Get number of iterations
    n.getParam(ros::this_node::getName() + "/iterations", iterations);
    n.getParam(ros::this_node::getName() + "/clear_costmaps", clear_costmaps);

    //wait for the action server to come up
    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //wait for services
    costmap_service.waitForExistence();
    goal_service.waitForExistence();

    std_srvs::Empty service_msg;

    //clear the costmaps
    if(clear_costmaps){
        if (!costmap_service.call(service_msg))
        {
            ROS_WARN("Failed to clear costmaps");
        }
    }
    //Call the goal service to recieve the first goal
    if (!goal_service.call(service_msg))
    {
        ROS_ERROR("Failed to request for a goal");
    }

    //Keeps the node running and performs necessary updates
    ros::spin();

    return 0;
}
