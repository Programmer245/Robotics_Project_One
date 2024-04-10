/**
 * Computes odometry from gps data. 
 * Subscribes to /fix topic to get gps data as sensor_msgs/NavSatFix objects.
 * Converts sensor_msgs/NavSatFix objects to nav_msgs/Odometry objects.
 * Publishes converted data to gps/odom topic
 * 
 * @author anto
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

class pub_sub {
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
    public:
        pub_sub() {
            sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
            pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        void callback(const sensor_msgs::NavSatFix::ConstPtr &data) {
            ROS_INFO("Callback triggered.");
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    pub_sub my_pub_sub;
    ros::spin();
    return 0;
}