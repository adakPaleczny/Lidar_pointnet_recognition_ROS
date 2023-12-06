#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZ;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id ) {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    
    pcl::fromROSMsg(*cloud_msg, pc_curr); // Failed to find match for field 'intensity'.

    PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    
    pcl_conversions::toPCL(cloud_msg->header, pc_curr.header);
    pcl_conversions::toPCL(cloud_msg->header, pc_ground.header);
    pcl_conversions::toPCL(cloud_msg->header, pc_non_ground.header);
    ROS_INFO_THROTTLE(20, "Result: Input PointCloud: %d -> Ground/ NonGround: %d/%d. Running_time: %f sec", pc_curr.size(), pc_ground.size(), pc_non_ground.size(), time_taken);

    // pub_cloud.publish(cloud2msg(pc_curr));
    // pub_ground.publish(cloud2msg(pc_ground));
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.frame_id));
}

int main(int argc, char**argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<string>("/cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&nh));

    pub_cloud       = nh.advertise<sensor_msgs::PointCloud2>("/demo/cloud", 1);
    pub_ground      = nh.advertise<sensor_msgs::PointCloud2>("/demo/ground", 1);
    pub_non_ground  = nh.advertise<sensor_msgs::PointCloud2>("/demo/nonground", 1);
    
    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 1, callbackCloud);


    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
