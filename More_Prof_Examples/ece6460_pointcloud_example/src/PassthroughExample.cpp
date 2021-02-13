#include "PassthroughExample.hpp"
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

namespace ece6460_pointcloud_example
{

  PassthroughExample::PassthroughExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    srv_.setCallback(boost::bind(&PassthroughExample::reconfig, this, _1, _2));  // binds reconfigure to get updated when new params

    sub_cloud_ = n.subscribe("points", 10, &PassthroughExample::recvCloud, this);  // Normally 1 here, but processing time can vary, don't loose any PC's
    pub_filtered_cloud_ = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
  }

  void PassthroughExample::reconfig(PointCloudExampleConfig& config, uint32_t level)  // Can't directly process on ROS point cloud. Convert to PCL pointcloud
  {
    cfg_ = config;  // cfg_ global in class, declared in hpp
  }

  // Talk raw point cloud and run pass through on it
  void PassthroughExample::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Instantiate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Copy ROS message data into PCL cloud
    pcl::fromROSMsg(*msg, *input_cloud);

    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices(new std::vector <int>);  // Point cloud is big array of points, roi = region of interest. Contains a subset of the original array
    pcl::PassThrough<pcl::PointXYZ> pass;  // Finds x, y, z of points. Stores points that satisfy roi. Matches type of input/filtered cloud

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud(input_cloud); // Already wants a pointer to a cloud, so no dereferrence

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName("x"); // 3 fields b/c PointXYZ
    pass.setFilterLimits(cfg_.x_min, cfg_.x_max);
    pass.filter(*roi_indices);  // roi_indices refers to point in input cloud. it's a subset that satisfys x value between x_min and x_max

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices(roi_indices);  // From result of x limits, check y values of those points 
    pass.setFilterFieldName("y");
    pass.setFilterLimits(cfg_.y_min, cfg_.y_max);
    pass.filter(*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices(roi_indices);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(cfg_.z_min, cfg_.z_max);
    pass.filter(*filtered_cloud);  // (This is a pointer) final stage so pass filter cloud so instead of integer array it's a corresponding point cloud

    // Copy filtered cloud data into a ROS message
    sensor_msgs::PointCloud2 output_msg;  
    pcl::toROSMsg(*filtered_cloud, output_msg);  // (pcl::PointCloud, sensor_msgs::PointCloud2)

    // Publish output point cloud
    pub_filtered_cloud_.publish(output_msg);
   
  }

}
