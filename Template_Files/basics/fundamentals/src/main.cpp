// ROS and node class header file
#include <ros/ros.h>
#include "fundamentals.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "fundamentals");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  basics::Fundamentals node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
