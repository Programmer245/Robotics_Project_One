/**
 * Converts odometry to tf
 * Subscribes to /input_odom to get the odometry data as nav_msgs/Odometry objects
 * Converts nav_msgs/Odometry to tf
 * Publishes tf on /tf topic
 * 
 * Code taken from tf/src/pub.cpp
 * 
 * @author anto, abdo, jie
*/

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub {
    private:
        ros::NodeHandle n;
        tf::TransformBroadcaster br;
        ros::Subscriber sub;

        std::string *root_frame; // Stores the root_frame and child_frame cmd line arguments
        std::string *child_frame;
    public:
        tf_sub_pub(std::string *s1, std::string *s2) {
            root_frame = s1;
            child_frame = s2;
			ROS_INFO("Launched node with root frame : %s, child frame : %s", (*root_frame).c_str(), (*child_frame).c_str()); 
            sub = n.subscribe("/input_odom", 1000, &tf_sub_pub::callback, this);
        }

        void callback(const nav_msgs::Odometry::ConstPtr &data) {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(data->pose.pose.position.x, data->pose.pose.position.y, 0));
            tf::Quaternion q;

            tf::Pose pose; // Get the yaw from the odometry message: https://answers.ros.org/question/41233/how-to-understand-robot-orientation-from-quaternion-yaw-angle/
            tf::poseMsgToTF(data->pose.pose, pose);
            double yaw_angle = tf::getYaw(pose.getRotation());
            ROS_INFO("Yaw: %lf", yaw_angle);

            q.setRPY(0, 0, yaw_angle);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), *root_frame, *child_frame));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_tf");
    std::string str1(argv[1]);
    std::string str2(argv[2]);
    tf_sub_pub my_tf_sub_pub(&str1, &str2); // Initialize instance of tf_sub_pub by passing it cmd line args
    ros::spin();
    return 0;
}