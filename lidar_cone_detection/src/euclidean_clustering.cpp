#include "euclidean_clustering.h"

Lidar_clustering::Lidar_clustering(ros::NodeHandle* nh){
    int check = 0;

    // Params
    std::string pointcloud_topic_;

    check += !nh->getParam("euclidean_clustering/print_fps", print_fps_);
    check += !nh->getParam("euclidean_clustering/z_axis_min", z_axis_min_);
    check += !nh->getParam("euclidean_clustering/z_axis_max", z_axis_max_);
    check += !nh->getParam("euclidean_clustering/x_axis_min", x_axis_min_);
    check += !nh->getParam("euclidean_clustering/x_axis_max", x_axis_max_);
    check += !nh->getParam("euclidean_clustering/y_axis_min", y_axis_min_);
    check += !nh->getParam("euclidean_clustering/y_axis_max", y_axis_max_);
    check += !nh->getParam("euclidean_clustering/tolerance", tolerance_);
    check += !nh->getParam("euclidean_clustering/cluster_size_min", cluster_size_min_);
    check += !nh->getParam("euclidean_clustering/cluster_size_max", cluster_size_max_);

    check += !nh->getParam("pointcloud_topic", pointcloud_topic_);


    if (check > 0)
        ROS_ERROR("[EUCLIDEAN CLUSTERING] Parameters loaded incorrectly! %d", check);
    else
        ROS_INFO("[EUCLIDEAN CLUSTERING] Parameters loaded correctly");

    point_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/demo/nonground", 1, &Lidar_clustering::pointCloudCallback, this);

    cluster_array_pub_ = nh->advertise<lidar_cone_detection::Clusters>("clusters", 100);
    cloud_filtered_pub_ = nh->advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
    pose_array_pub_ = nh->advertise<geometry_msgs::PoseArray>("poses", 100);
    marker_array_pub_ = nh->advertise<visualization_msgs::MarkerArray>("markers", 100);
}

bool Lidar_clustering::isPossibleConeCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cluster){
  return true;
}

void Lidar_clustering::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in) {
  ros::WallTime start_time = ros::WallTime::now();
  
  
  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
  
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
  
  /*** Remove ground and ceiling ***/
  pcl::IndicesPtr pc_indices_z(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(pcl_pc_in);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(z_axis_min_, z_axis_max_);
  pt.filter(*pc_indices_z);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_filtring(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*pcl_pc_in, *pc_indices_z, *pcl_pc_filtring);

  pcl::IndicesPtr pc_indices_zx(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pt2;
  pt2.setInputCloud(pcl_pc_filtring);
  pt2.setFilterFieldName("x");
  pt2.setFilterLimits(x_axis_min_, x_axis_max_);
  pt2.filter(*pc_indices_zx);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_filtring2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*pcl_pc_filtring, *pc_indices_zx, *pcl_pc_filtring2);


  pcl::IndicesPtr pc_indices_zxy(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pt3;
  pt3.setInputCloud(pcl_pc_filtring2);
  pt3.setFilterFieldName("y");
  pt3.setFilterLimits(y_axis_min_, y_axis_max_);
  pt3.filter(*pc_indices_zxy);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_filtring3(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*pcl_pc_filtring2, *pc_indices_zxy, *pcl_pc_filtring3);

  /*** Euclidean clustering ***/
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(pcl_pc_filtring2);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(tolerance_);
  ec.setMinClusterSize(cluster_size_min_);
  ec.setMaxClusterSize(cluster_size_max_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_pc_filtring2);
  ec.setIndices(pc_indices_zxy);
  ec.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;
  
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cluster->points.push_back(pcl_pc_filtring2->points[*pit]);
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
   
    //Checking if it is a cone by checking max size of cluster 
    if(isPossibleConeCluster(cluster)){
      clusters.push_back(cluster);
    }
        
  }
  
  /*** Output ***/
  // if(cloud_filtered_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::copyPointCloud(*pcl_pc_filtring2, *pc_indices_zxy, *pcl_pc_out);
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    cloud_filtered_pub_.publish(ros_pc2_out);
  // }
  
  lidar_cone_detection::Clusters cluster_array;
  geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;
  
  for(int i = 0; i < clusters.size(); i++) {
    if(cluster_array_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*clusters[i], ros_pc2_out);
      cluster_array.clusters.push_back(ros_pc2_out);
    }
    
    if(pose_array_pub_.getNumSubscribers() > 0) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters[i], centroid);
      
      geometry_msgs::Pose pose;
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);
      
#ifdef LOG
      Eigen::Vector4f min, max;
      pcl::getMinMax3D(*clusters[i], min, max);
      std::cerr << ros_pc2_in->header.seq << " "
		<< ros_pc2_in->header.stamp << " "
		<< min[0] << " "
		<< min[1] << " "
		<< min[2] << " "
		<< max[0] << " "
		<< max[1] << " "
		<< max[2] << " "
		<< std::endl;
#endif
    }
    
    if(marker_array_pub_.getNumSubscribers() > 0) {
      Eigen::Vector4f min, max;
      pcl::getMinMax3D(*clusters[i], min, max);
      
      visualization_msgs::Marker marker;
      marker.header = ros_pc2_in->header;
      marker.ns = "euclidean_clustering";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      
      geometry_msgs::Point p[24];
      p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
      p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
      p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
      p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
      p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
      p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
      p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
      p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
      p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
      p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
      for(int i = 0; i < 24; i++) {
  	marker.points.push_back(p[i]);
      }
      
      marker.scale.x = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.lifetime = ros::Duration(1);
      marker_array.markers.push_back(marker);
    }
  }
  
  if(cluster_array.clusters.size()) {
    cluster_array.header = ros_pc2_in->header;
    cluster_array_pub_.publish(cluster_array);
  }

  if(pose_array.poses.size()) {
    pose_array.header = ros_pc2_in->header;
    pose_array_pub_.publish(pose_array);
  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
  
    if(print_fps_){
      ROS_INFO_THROTTLE(10, "[DV_CONE_DETECTOR] EUCLIDEAN CLUSTERING -> Duratoin: %.2f ms", (ros::WallTime::now() - start_time).toNSec() * 1e-6);
    }
  }