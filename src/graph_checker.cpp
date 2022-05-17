#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <unistd.h>

bool got_graph = false;

void callback(tuw_multi_robot_msgs::Graph msg){
    ROS_INFO("Received the graph");
    got_graph = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "graph_checker");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/segments", 1000, callback);

    ros::Rate rate(10);
    while(ros::ok()){
        if(got_graph){
            ROS_INFO("Graph is received");
            ros::shutdown();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}