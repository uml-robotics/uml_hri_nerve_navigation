/*
    Publishes the robot's position in a geometry_msgs/Pose message based on the TF2
    difference between map and base_link.
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// Creates and runs the robot_pose_publisher node.
int main(int argc, char **argv)
{
    // initialize ROS and the node
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // configuring parameters
    std::string map_frame, base_frame;
    double publish_frequency;
    bool is_stamped;
    ros::Publisher p_pub;

    //get param values
    nh_priv.param<std::string>("map_frame", map_frame, "map");
    nh_priv.param<std::string>("base_frame", base_frame, "base_link");
    nh_priv.param<double>("publish_frequency", publish_frequency, 10);
    nh_priv.param<bool>("is_stamped", is_stamped, false);

    //determine if the timestamp header is needed
    if (is_stamped)
        p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    else
        p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

    //create the listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);

    ros::Rate rate(publish_frequency);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transform;
        try
        {
            //find the transform
            transform = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0));

            //construct a pose message
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map_frame;
            pose_stamped.header.stamp = ros::Time::now();

            pose_stamped.pose.orientation.x = transform.transform.rotation.x;
            pose_stamped.pose.orientation.y = transform.transform.rotation.y;
            pose_stamped.pose.orientation.z = transform.transform.rotation.z;
            pose_stamped.pose.orientation.w = transform.transform.rotation.w;

            pose_stamped.pose.position.x = transform.transform.translation.x;
            pose_stamped.pose.position.y = transform.transform.translation.y;
            pose_stamped.pose.position.z = transform.transform.translation.z;
            
            //publish the transform
            if (is_stamped)
                p_pub.publish(pose_stamped);
            else
                p_pub.publish(pose_stamped.pose);
        }
        catch (tf2::TransformException &ex)
        {
            // just continue on
        }

        rate.sleep();
    }
    return 0;
}