#ifndef EUCLIDEAN_CLUSTERING_H
#define EUCLIDEAN_CLUSTERING_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "dv_interfaces/ClusterArray.h"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


class Lidar_clustering{
public:
    Lidar_clustering(ros::NodeHandle* nh);

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in);
private:
    // Subscribers 
    ros::Subscriber point_cloud_sub; // = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, pointCloudCallback);

    // ROS Publishers
    ros::Publisher cluster_array_pub_;
    ros::Publisher cloud_filtered_pub_;
    ros::Publisher pose_array_pub_;
    ros::Publisher marker_array_pub_;

    bool print_fps_;
    float z_axis_min_;
    float z_axis_max_;
    float x_axis_min_;
    float x_axis_max_;
    float y_axis_min_;
    float y_axis_max_;
    float tolerance_;
    int cluster_size_min_;
    int cluster_size_max_;

    bool isPossibleConeCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
};


#endif /* EUCLIDEAN_CLUSTERING_H */