//
// Created by zero on 4/12/24.
//
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

std::string framePar;

class lidar_pub_sub {

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
    public:
        lidar_pub_sub(){
            sub = n.subscribe("/os_cloud_node/points", 1, &lidar_pub_sub::callback, this);
            pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

            dynamic_reconfigure::Server<first_project::parametersConfig> server;
            dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

            f = boost::bind(&callbackParameter, _1, _2);
            server.setCallback(f);
        }
        void callback(const sensor_msgs::PointCloud2::ConstPtr &data) {
            //Todo
            ROS_INFO ("Good Evening!");
        }
        void static callbackParameter(first_project::parametersConfig &config, uint32_t level) {
            /*ROS_INFO("Reconfigure Request: %d %f %s %s %d",
                      config.int_param, config.double_param,
                      config.str_param.c_str(),
                      config.bool_param?"True":"False",
                      config.size);*/
            framePar = config.frame;
            ROS_INFO ("Frame parameter: %s", framePar.c_str());

            ROS_INFO ("%d",level);
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    lidar_pub_sub my_lidar_pub_sub;
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}