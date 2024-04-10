/**
 * Computes odometry from gps data. 
 * Subscribes to /fix topic to get gps data as sensor_msgs/NavSatFix objects.
 * Converts sensor_msgs/NavSatFix objects to nav_msgs/Odometry objects.
 * Publishes converted data to gps/odom topic
 * 
 * Remember to start the launch file first (to get the initial parameters)
 * 
 * @author anto, abdo
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>

double a = 6378137; // Equatorial radius in m
double b = 6356752; // Polar radius in m
double e_square = 1-pow(a/b,2); // First numerical eccentricity

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

            initialECEF = AltLonLatToECEF(startAlt, startLon, startLat); // Convert the initial altitude, longitude, and latitude to ECEF

            sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
            pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        void callback(const sensor_msgs::NavSatFix::ConstPtr &data) {
            std::array<double, 3> newCoordinates = AltLonLatToECEF(data->altitude, data->longitude, data->latitude); // Gets the new ECEF coordinates
        }

        //The 3 parameters of this method are in degrees
        std::array<double, 3> AltLonLatToECEF(double alt, double lon, double lat) {
            ROS_INFO("Received parameters: alt: %lf, long: %lf, lat: %lf", alt, lon, lat); // TODO: Must delete

            std::array<double, 3> res;

            double latRad = lat * M_PI/180;
            double lonRad = lon * M_PI/180;

            double nResult = N(latRad);

            res[0] = (nResult + alt) * cos(latRad) * cos(lonRad);
            res[1] = (nResult + alt) * cos(latRad) * sin(lonRad);
            res[2] = (nResult * (1 - e_square) + alt) * sin(latRad);

            ROS_INFO("Converted parameters: X: %lf, Y: %lf, Z: %lf", res[0], res[1], res[2]); // TODO: Must delete

            return res; // Contains XECEF, YECEF, ZECEF
        }

        //The parameter is in radiants
        double N(double latRad) {
            return a/sqrt(1-e_square*pow(sin(latRad),2));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    pub_sub my_pub_sub;
    ros::spin();
    return 0;
}