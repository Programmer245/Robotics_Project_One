/**
 * Changes header on lidar data.
 * Subscribes to /os_cloud_node/points to get lidar data as sensor_msgs/PointCloud2 objects.
 * Changes header of sensor_msgs/PointCloud2 object.
 * Publishes updated data to /pointcloud_remapped
 * 
 * Useful links:
 * https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743
 * http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
 * 
 * @author anto, abdo, jie
*/

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
        }
        void callback(const sensor_msgs::PointCloud2::ConstPtr &data) {
            sensor_msgs::PointCloud2 new_msg; 

            new_msg.header.seq = data->header.seq;
            new_msg.header.stamp = data->header.stamp;
            new_msg.header.frame_id = framePar;

            new_msg.height = data->height;
            new_msg.width = data->width;
            new_msg.fields = data->fields;
            new_msg.is_bigendian = data->is_bigendian;
            new_msg.point_step = data->point_step;
            new_msg.row_step = data->row_step;
            new_msg.data = data->data;
            new_msg.is_dense = data->is_dense;

            pub.publish(new_msg);
        }
};

void callbackParameter(first_project::parametersConfig &config, uint32_t level) {
    framePar = config.frame;
    ROS_INFO("Parameter change: %s", config.frame.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

    f = boost::bind(&callbackParameter, _1, _2);
    server.setCallback(f);

    lidar_pub_sub my_lidar_pub_sub;

    ros::spin();
    return 0;
}