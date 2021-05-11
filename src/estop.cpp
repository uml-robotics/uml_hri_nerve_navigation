//This is a simple estop node that publishes 0 to cmd_vel

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher vel_pub;

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "robot_estop");
  ros::NodeHandle n;

  //Create the publisher object
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000, false);

  //Warn that estop has been activated
  ROS_WARN("Estop has been activated");
  ROS_WARN("Estop has been activated");
  ROS_WARN("Estop has been activated");
  
  while(ros::ok())
  {
      //Set twist to have 0 velocity
      geometry_msgs::Twist twist;
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.linear.z = 0;
      twist.angular.x = 0;
      twist.angular.y = 0;
      twist.angular.z = 0;

      vel_pub.publish(twist);
  }

  return 0;
}
