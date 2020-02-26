// Include guard to prevent multiple declarations
#ifndef DEADRECKONING_H
#define DEADRECKONING_H

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// Namespace matches ROS package name
namespace dead_reckoning{

class DeadReckoning
{
public:
  DeadReckoning(ros::NodeHandle n, ros::NodeHandle pn);
  
private:
  void timerCallback(const ros::TimerEvent& event);
  ros::Timer timer;
  ros::Publisher pub_point;
  
  double x;
  double y;
  double psi;
  
};

}

#endif // DEADRECKONING_H

