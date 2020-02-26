// Header file for the class
#include "DeadReckoning.h"

// Namespace matches ROS package name
namespace dead_reckoning {

// Constructor with global and private node handle arguments
DeadReckoning::DeadReckoning(ros::NodeHandle n, ros::NodeHandle pn)
{
    timer = n.createTimer(ros::Duration(0.02), &DeadReckoning::timerCallback, this);
    pub_point = n.advertise<geometry_msgs::Point>("point",1);
    x = 0;
    y = 0;
    psi = 0;
}

void DeadReckoning::timerCallback(const ros::TimerEvent& event){
    double v = 1.0;
    double pdot = 0.1;
    
    x = x + 0.02 * v * cos(psi);
    y = y + 0.02 * v * sin(psi);
    psi = psi + 0.02 * pdot;
    
    geometry_msgs::Point point_msg;
    point_msg.x = x;
    point_msg.y = y;
    pub_point.publish(point_msg);

}


  
}
