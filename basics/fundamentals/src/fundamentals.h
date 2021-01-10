// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>

// Namespace matches ROS package name
namespace basics{

  class Fundamentals
  {
  public:
    Fundamentals(ros::NodeHandle n, ros::NodeHandle pn);
        
  private:
    ros::Timer timer;
    void timerCallback(const ros::TimerEvent& event);
    int counter = 10; 

  };

}


