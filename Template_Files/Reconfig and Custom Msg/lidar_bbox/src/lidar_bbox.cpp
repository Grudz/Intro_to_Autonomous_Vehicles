
// Header file for the class
#include "lidar_bbox.h"


// Namespace matches ROS package name
namespace lidar_bbox {

  // Constructor with global and private node handle arguments
  BBox::BBox(ros::NodeHandle n, ros::NodeHandle pn)
  {
    srv_.setCallback(boost::bind(&BBox::reconfig, this, _1, _2));  // binds reconfigure to get updated when new params
    sub_bbox_ = n.subscribe("bbox_points", 1, &BBox::recvBBoxInfo, this);

  }  

  void BBox::reconfig(PointCloudbboxConfig& config, uint32_t level)  
  {
    cfg_ = config;  // cfg_ global in class, declared in hpp
  }

  void BBox::recvBBoxInfo(const lidar_filter::cluster_info& info)
  {
    // PUT THESE INTO A STD::VECTOR B/C Points being dropped
      double min_x;
      double min_y;
      double min_z;
      double max_x;
      double max_y;
      double max_z;
      
      min_x = info.min_x;
      min_y = info.min_y;
      min_z = info.min_z;
      max_x = info.max_x;
      max_y = info.max_y;
      max_z = info.max_z;   
                          
      ROS_INFO("\n ----- LOOP START ----- \n"); 
      ROS_INFO("Min X = %f\n", min_x);                            
      ROS_INFO("Min y = %f\n", min_y);                            
      ROS_INFO("Min z = %f\n", min_z); 
      ROS_INFO("Max X = %f\n", max_x);                            
      ROS_INFO("Max y = %f\n", max_y);                            
      ROS_INFO("Max z = %f\n", max_z); 

      // pass variables into and call box drawing function

  }

}
