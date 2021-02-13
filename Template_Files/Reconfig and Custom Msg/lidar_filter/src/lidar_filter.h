// Include guard to prevent multiple declarations
#pragma once

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

// Dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>  
#include <lidar_filter/PointCloudFilterConfig.h>


// Message headers
#include <sensor_msgs/PointCloud2.h>  // Message definition
#include <pcl_conversions/pcl_conversions.h>  // Allows converation to ROS message
#include <geometry_msgs/PoseArray.h>
#include "lidar_filter/cluster_info.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>  // copy/paste point cloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

// Bounding Box message type: https://pointclouds.org/documentation/structpcl_1_1_bounding_box_x_y_z.html
// Tutorial: https://www.programmersought.com/article/90624493201/
// ROS Wiki: http://wiki.ros.org/pcl_ros#ROS_nodelets
// Github of Rotated boxes: https://github.com/LidarPerception/object_builders_lib/blob/master/src/min_box_object_builder.cpp
//#include <pcl/recognition/linemod/line_rgbd.h>

// Namespace matches ROS package name
namespace lidar_filter{

  class BBoxPrep
  {
  public:
  
    BBoxPrep(ros::NodeHandle n, ros::NodeHandle pn);
        
  private:

    void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);  // Subscribe to point cloud
    void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out); // Passthrough filter function
    void reconfig(PointCloudFilterConfig& config, uint32_t level);  // Called on updated gui

    ros::Subscriber sub_cloud_;
    ros::Publisher pub_passthrough_cloud_;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_bbox_;

    // Publishing bounding box message
    jsk_recognition_msgs::BoundingBoxArray bbox_array_;
    
    // Dynamic reconfig stuff
    dynamic_reconfigure::Server<PointCloudFilterConfig> srv_;
    PointCloudFilterConfig cfg_;

    // KD nearest nieghbors search tree for point clouds
    pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;  

  };

}


