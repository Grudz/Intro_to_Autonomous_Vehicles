// ROS and node class header file
#include "lidar_filter.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lidar_filter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  lidar_filter::BBoxPrep node(n, pn);

  /*// Multithreaded, 0 = as many threads as needed
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // Since multithreaded, do this
  ros::waitForShutdown(); */
  ros::spin();
}
