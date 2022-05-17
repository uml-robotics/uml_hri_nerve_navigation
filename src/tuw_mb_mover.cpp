#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float goal_x = 0.0;
float goal_y = 0.0;
float orientation_w = 0.0;

void callback(const geometry_msgs::PoseStampedPtr& msg){
    geometry_msgs::PoseStamped goal = *msg;
    if (goal_x == 0) goal_x = goal.pose.position.x;
    if (goal_y == 0) goal_y = goal.pose.position.y;
    if (orientation_w == 0) orientation_w = goal.pose.orientation.w;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_base_mover_node");
    ros::NodeHandle nh;

    std::string robot_name = "pioneer";
    nh.getParam(ros::this_node::getName()+"/robot_name",robot_name);
   
    std::string action_name = robot_name+"_move_base";
    std::string status_topic = robot_name+"/test_status";
    std::string goal_topic = robot_name+"/goal";

    MoveBaseClient ac(action_name, true);
    
    ros::Publisher test_status_pub = nh.advertise<std_msgs::Bool>(status_topic, 1000, true);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Pose2D>(goal_topic, 1000, true);
    ros::Subscriber goal_pos_subscriber = nh.subscribe(robot_name+"/goal_pose", 1000, callback);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    nh.getParam(ros::this_node::getName()+"/goal_x",goal_x);
    nh.getParam(ros::this_node::getName()+"/goal_y",goal_y);
    nh.getParam(ros::this_node::getName()+"/orientation_w",orientation_w);

    geometry_msgs::Pose2D goal_pose;
    goal_pose.x = goal_x;
    goal_pose.y = goal_y;
    goal_pub.publish(goal_pose);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.w = orientation_w;

    std_msgs::Bool boolean;
    boolean.data = true;
    test_status_pub.publish(boolean);

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal is reached!");
    }
    else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Goal aborted");
    }
    else { 
        ROS_INFO("Robot failed to move");
    }

    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}