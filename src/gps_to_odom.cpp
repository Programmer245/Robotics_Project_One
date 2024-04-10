/**
 * Computes odometry from gps data. 
 * Subscribes to /fix topic to get gps data as sensor_msgs/NavSatFix objects.
 * Converts sensor_msgs/NavSatFix objects to nav_msgs/Odometry objects.
 * Publishes converted data to gps/odom topic
 * 
 * Remember to start the launch file first (to get the initial parameters)
 * 
 * @author anto, abdo, jie
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <tf/transform_datatypes.h>

double a = 6378137; // Equatorial radius in m
double b = 6356752; // Polar radius in m
double e_square = 1-pow(a/b,2); // First numerical eccentricity

class pub_sub {
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;

        std::array<double, 3> initialECEF; // Stores the initial X,Y,Z ECEF coordinates of the robot
        std::array<double, 3> initialAltLonLat; // Stores the initial altitude, longitude, and latitude of the robot in degrees
        std::array<double, 3> oldENU; // The X,Y,Z ENU of the previous position 
    public:
        pub_sub() {
            n.getParam("alt_r", initialAltLonLat[0]); // Reads the parameters containing the initial altitude, longitude, and latitude of the robot
            n.getParam("lon_r", initialAltLonLat[1]);
            n.getParam("lat_r", initialAltLonLat[2]);

            initialECEF = AltLonLatToECEF(initialAltLonLat[0], initialAltLonLat[1], initialAltLonLat[2]); // Convert the initial altitude, longitude, and latitude to ECEF
            std::fill(std::begin(oldENU), std::end(oldENU), 0); // Initial old position set to 0

            sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
            pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        void callback(const sensor_msgs::NavSatFix::ConstPtr &data) {
            std::array<double, 3> newCoordinatesECEF = AltLonLatToECEF(data->altitude, data->longitude, data->latitude); // Gets the new ECEF coordinates
            std::array<double, 3> newCoordinatesENU = ECEFToENU(newCoordinatesECEF[0], newCoordinatesECEF[1], newCoordinatesECEF[2]); // Gets the new ENU coordinates

            nav_msgs::Odometry odom_msg; // Create new odometry message

            odom_msg.pose.pose.position.x = newCoordinatesENU[0]; // Set message pose (x,y,z)
            odom_msg.pose.pose.position.y = newCoordinatesENU[1];
            odom_msg.pose.pose.position.z = newCoordinatesENU[2];

            // Need to set orientation (turtle strat)

            double theta = atan((newCoordinatesENU[1]-oldENU[1])/(newCoordinatesENU[0]-oldENU[0])); // Calculates the heading by look at the previous and current position
            ROS_INFO("Heading: %lf", theta);

            /*
            tf::Quaternion q; // Create tf quaternion
            q.setRPY(0, 0, theta); // Set orientation of quaternion

            geometry_msgs::Quaternion q2; // Create other type of quaternion
            quaternionMsgToTF(q2, q); // Create geometry_msgs quaternion from tf quaternion

            odom_msg.pose.pose.orientation = q2;

            pub.publish(odom_msg);
            */

            oldENU = newCoordinatesENU; // We update the previous ENU
        }

        //The 3 parameters of this method are in degrees
        std::array<double, 3> AltLonLatToECEF(double alt, double lon, double lat) {
            ROS_INFO("Received parameters in AltLonLatToECEF: alt: %lf, long: %lf, lat: %lf", alt, lon, lat); // TODO: Must delete

            std::array<double, 3> res;

            double latRad = lat * M_PI/180;
            double lonRad = lon * M_PI/180;

            double nResult = N(latRad);

            res[0] = (nResult + alt) * cos(latRad) * cos(lonRad);
            res[1] = (nResult + alt) * cos(latRad) * sin(lonRad);
            res[2] = (nResult * (1 - e_square) + alt) * sin(latRad);

            ROS_INFO("Converted parameters in AltLonLatToECEF: X: %lf, Y: %lf, Z: %lf", res[0], res[1], res[2]); // TODO: Must delete

            return res; // Contains XECEF, YECEF, ZECEF
        }

        std::array<double, 3> ECEFToENU(double X, double Y, double Z) {
            ROS_INFO("Received parameters in ECEFToENU: X: %lf, Y: %lf, Z: %lf", X, Y, Z); // TODO: Must delete

            std::array<double, 3> res;

            double deltaX = X-initialECEF[0]; // Xp - Xr
            double deltaY = Y-initialECEF[1];
            double deltaZ = Z-initialECEF[2];

            res[0] = -sin(initialAltLonLat[1])*deltaX + cos(initialAltLonLat[1])*deltaY;
            res[1] = -sin(initialAltLonLat[2])*cos(initialAltLonLat[1])*deltaX - sin(initialAltLonLat[2])*sin(initialAltLonLat[1])*deltaY + cos(initialAltLonLat[2])*deltaZ;
            res[2] = cos(initialAltLonLat[2])*cos(initialAltLonLat[1])*deltaX + cos(initialAltLonLat[2])*sin(initialAltLonLat[1])*deltaY + sin(initialAltLonLat[2])*deltaZ;

            ROS_INFO("Converted parameters in ECEFToENU: X: %lf, Y: %lf, Z: %lf", res[0], res[1], res[2]); // TODO: Must delete

            return res; // Contains ENU coordinates
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