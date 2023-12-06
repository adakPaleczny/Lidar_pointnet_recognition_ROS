#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "euclidean_clustering.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "dv_cone_detector/euclidean_clustering");

    ros::NodeHandle nh("~");

    Lidar_clustering Lidar_clust(&nh);

    ros::spin();

    return 0;
}
