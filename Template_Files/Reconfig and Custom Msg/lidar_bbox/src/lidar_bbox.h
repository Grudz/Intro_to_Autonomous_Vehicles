// Include guard to prevent multiple declarations
#pragma once

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>  // Dynamic reconfigure stuff
#include <lidar_bbox/PointCloudbboxConfig.h>
#include "lidar_filter/cluster_info.h"

// Message headers
#include <sensor_msgs/PointCloud2.h>  // Message definition
#include <pcl_conversions/pcl_conversions.h>  // Allows converation to ROS message
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h> // Publish markers

// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>
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
// https://answers.ros.org/question/294054/design-a-bounding-box-in-rviz-using-poses-information/
//#include <pcl/recognition/linemod/line_rgbd.h>

/*// PCL and PCL to ROS includes
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/random_sample.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>

// ROS inludes
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
*/

// Namespace matches ROS package name
namespace lidar_bbox{

  class BBox
  {
  public:
  
    BBox(ros::NodeHandle n, ros::NodeHandle pn);
        
  private:

    void recvBBoxInfo(const lidar_filter::cluster_info& info);  // Subscribe to point cloud
    void reconfig(PointCloudbboxConfig& config, uint32_t level);  // Called on updated gui

    ros::Subscriber sub_bbox_;

    dynamic_reconfigure::Server<PointCloudbboxConfig> srv_;
    PointCloudbboxConfig cfg_;

  };

}


