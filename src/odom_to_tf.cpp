#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <string>



class tf_sub_pub {
    private:
        ros::NodeHandle n;
        tf::TransformBroadcaster br;
        ros::Subscriber sub;
    public:
        tf_sub_pub(char * root_frame, char * child_frame ) {
			ROS_INFO("Root frame : %s, Child frame : %s", root_frame, child_frame);
            sub = n.subscribe("/input_odom", 1000, &tf_sub_pub::callback, this);
        }

        void callback(const nav_msgs::Odometry::ConstPtr &data) {
            tf::Transform transform;
            // TODO
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_tf");
    tf_sub_pub my_tf_sub_pub(argv[1], argv[2]);
    ros::spin();
    return 0;
}