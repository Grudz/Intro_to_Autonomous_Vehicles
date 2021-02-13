// ROS and node class header file
#include "lidar_bbox.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lidar_bbox");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  lidar_bbox::BBox node(n, pn);

  /*// Multithreaded, 0 = as many threads as needed
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // Since multithreaded, do this
  ros::waitForShutdown(); */
  ros::spin();
}
