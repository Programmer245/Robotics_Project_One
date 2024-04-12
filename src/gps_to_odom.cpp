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

const double a = 6378137; // Equatorial radius in m
const double b = 6356752; // Polar radius in m
const double e_square = 1-pow((double)(b/a),2); // First numerical eccentricity

class pub_sub {
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;

        std::array<double, 3> initialECEF; // Stores the initial X,Y,Z ECEF coordinates of the robot in meters
        std::array<double, 3> initialAltLonLat; // Stores the initial altitude (meters), longitude (radians), and latitude (radians) of the robot 
        std::array<double, 3> oldENU; // Stores the X,Y,Z ENU coordinates of the previous position in meters
    public:
        pub_sub() {
            n.getParam("alt_r", initialAltLonLat[0]); // Read initial alt, lon, lat parameters stored in launch file
            n.getParam("lon_r", initialAltLonLat[1]);
            n.getParam("lat_r", initialAltLonLat[2]);

            // Save the initial longitude and latitude as radians; this is done so that in the ENU function, we don't have to convert them every time we receive gps data
            initialAltLonLat[1] *= M_PI/180; 
            initialAltLonLat[2] *= M_PI/180; 

            initialECEF = AltLonLatToECEF(initialAltLonLat[0], initialAltLonLat[1]*(180/M_PI), initialAltLonLat[2]*(180/M_PI)); // Convert the initial altitude, longitude, and latitude to ECEF
            std::fill(std::begin(oldENU), std::end(oldENU), 0); // Set initial old ENU position to 0

            sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
            pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        /**
         * Callback function for the subscriber
         * 
         * @param data Data object containing the gps coordinates passed by reference
        */
        void callback(const sensor_msgs::NavSatFix::ConstPtr &data) {
            std::array<double, 3> newCoordinatesECEF = AltLonLatToECEF(data->altitude, data->longitude, data->latitude); // Gets the new ECEF coordinates
            std::array<double, 3> newCoordinatesENU = ECEFToENU(newCoordinatesECEF[0], newCoordinatesECEF[1], newCoordinatesECEF[2]); // Gets the new ENU coordinates

            nav_msgs::Odometry odom_msg; // Create new odometry message

            // Set the pose
            odom_msg.pose.pose.position.x = newCoordinatesENU[0]; // Set message pose (x,y,z)
            odom_msg.pose.pose.position.y = newCoordinatesENU[1];
            odom_msg.pose.pose.position.z = newCoordinatesENU[2];

            // Set the orientation
            // Calculates the heading in radians by looking at the current and previous position
            // Note that theta is in interval [-pi, pi]
            double theta = atan2(newCoordinatesENU[1]-oldENU[1],newCoordinatesENU[0]-oldENU[0]); 
            ROS_INFO("Heading: %lf", theta);

            tf::Quaternion q; // Create tf quaternion
            q.setRPY(0, 0, theta); // Set orientation of quaternion

            odom_msg.pose.pose.orientation.x = q.getX();
            odom_msg.pose.pose.orientation.y = q.getY();
            odom_msg.pose.pose.orientation.z = q.getZ();
            odom_msg.pose.pose.orientation.w = q.getW();

            pub.publish(odom_msg);

            oldENU = newCoordinatesENU; // We update the previous ENU
        }

        /**
         * Converts altitude, longitude, and latitude gps coordinates into ECEF X, Y, Z coordinates
         * 
         * @param alt Altitude in meters
         * @param lon Longitude in degrees
         * @param lat Latitude in degrees
         * 
         * @return An array of 3 doubles containing the X,Y,Z ECEF coordinates
        */
        std::array<double, 3> AltLonLatToECEF(double alt, double lon, double lat) {
            ROS_INFO("Received parameters in AltLonLatToECEF: alt: %lf, long: %lf, lat: %lf", alt, lon, lat); // TODO: Must delete

            std::array<double, 3> res; // Allocate new array of doubles in heap

            double lonRad = lon * M_PI/180; // Convert to radians
            double latRad = lat * M_PI/180; 

            double nResult = N(latRad); // Calculate N(phi) 

            res[0] = (nResult + alt) * cos(latRad) * cos(lonRad);
            res[1] = (nResult + alt) * cos(latRad) * sin(lonRad);
            res[2] = (nResult * (1 - e_square) + alt) * sin(latRad);

            ROS_INFO("Converted parameters in AltLonLatToECEF: X: %lf, Y: %lf, Z: %lf", res[0], res[1], res[2]); // TODO: Must delete

            return res; 
        }

        /**
         * Converts ECEF X,Y,Z coordinates into local X,Y,Z ENU coordinates
         * 
         * @param X X ECEF coordinate in meters
         * @param Y Y ECEF coordinate in meters
         * @param Z Z ECEF coordinate in meters
         * 
         * @return An array of 3 doubles containing the X, Y, Z ENU coordinates
        */
        std::array<double, 3> ECEFToENU(double X, double Y, double Z) {
            ROS_INFO("Received parameters in ECEFToENU: X: %lf, Y: %lf, Z: %lf", X, Y, Z); // TODO: Must delete

            std::array<double, 3> res; // Allocate new array of doubles in heap

            double deltaX = X-initialECEF[0]; // Xp - Xr
            double deltaY = Y-initialECEF[1]; // Yp - Yr
            double deltaZ = Z-initialECEF[2]; // Zp - Zr

            res[0] = -sin(initialAltLonLat[1])*deltaX + cos(initialAltLonLat[1])*deltaY;
            res[1] = -sin(initialAltLonLat[2])*cos(initialAltLonLat[1])*deltaX - sin(initialAltLonLat[2])*sin(initialAltLonLat[1])*deltaY + cos(initialAltLonLat[2])*deltaZ;
            res[2] = cos(initialAltLonLat[2])*cos(initialAltLonLat[1])*deltaX + cos(initialAltLonLat[2])*sin(initialAltLonLat[1])*deltaY + sin(initialAltLonLat[2])*deltaZ;

            ROS_INFO("Converted parameters in ECEFToENU: X: %lf, Y: %lf, Z: %lf", res[0], res[1], res[2]); // TODO: Must delete

            return res; // Contains ENU coordinates
        }

        /**
         * Calculates N(phi)
         * 
         * @param phi Latitude in radians 
         * @return N(phi) 
        */
        double N(double phi) {
            return a/sqrt(1-e_square*pow(sin(phi),2));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom"); // Initialize new ROS node named "gps_to_odom"
    pub_sub my_pub_sub;
    ros::spin();
    return 0;
}