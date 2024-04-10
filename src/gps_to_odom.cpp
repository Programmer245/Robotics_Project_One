/**
 * Computes odometry from gps data. 
 * Subscribes to /fix topic to get gps data as sensor_msgs/NavSatFix objects.
 * Converts sensor_msgs/NavSatFix objects to nav_msgs/Odometry objects.
 * Publishes converted data to gps/odom topic
 * 
 * @author anto, abdo
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>

#define a 6378137 // Equatorial radius

class pub_sub {
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        std::array<double, 3> initialECEF; // Stores the initial ECEF coordinates of the robot
    public:
        pub_sub() {
            double startAlt, startLon, startLat;

            n.getParam("alt_r", startAlt); // Reads the parameters containing the initial altitude, longitude, and latitude of the robot
            n.getParam("lon_r", startLon);
            n.getParam("lat_r", startLat);

            initialECEF = LatLonToECEF(startAlt, startLon, startLat); // Convert the initial altitude, longitude, and latitude to ECEF

            sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
            pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        void callback(const sensor_msgs::NavSatFix::ConstPtr &data) {
            std::array<double, 3> newCoordinates = LatLonToECEF(data->altitude, data->longitude, data->latitude); // Gets the new ECEF coordinates
            ROS_INFO("Callback triggered.");
        }

        //The 3 parameters of this method are in degrees
        std::array<double, 3> LatLonToECEF(double alt, double lon, double lat) {
            std::array<double, 3> res;

            double latRad = lat * M_PI/180;
            double lonRad = lon * M_PI/180;

            double nResult = N(latRad);

            res[0] = (nResult + alt) * cos(latRad) * cos(lonRad);
            res[1] = (nResult + alt) * cos(latRad) * sin(lonRad);
            res[2] = (nResult * (1 - exp(2)) + alt) * sin(latRad);

            return res; // Contains XECEF, YECEF, ZECEF
        }

        //The parameter is in radiants
        double N(double latRad) {
            return a/sqrt(1 - (exp(2) * pow(sin(latRad), 2)));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    pub_sub my_pub_sub();
    ros::spin();
    return 0;
}