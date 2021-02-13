#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>  // Message definition
#include <pcl_conversions/pcl_conversions.h>  // Allows converation to ROS message

#include <dynamic_reconfigure/server.h>  // Dynamic reconfigure stuff
#include <ece6460_pointcloud_example/PointCloudExampleConfig.h>

namespace ece6460_pointcloud_example
{

  class PassthroughExample
  {
    public:
      PassthroughExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(PointCloudExampleConfig& config, uint32_t level);  // Called on updated gui
      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);  // Calls when new lidar data

      ros::Publisher pub_filtered_cloud_;  // publishes filtered cloud
      ros::Subscriber sub_cloud_;  // sub raw cloud

      dynamic_reconfigure::Server<PointCloudExampleConfig> srv_;
      PointCloudExampleConfig cfg_;
  };

}
