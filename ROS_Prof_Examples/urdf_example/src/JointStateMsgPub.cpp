// Header file for the class
#include "JointStateMsgPub.h"

// Namespace matches ROS package name
namespace urdf_example {

// Constructor with global and private node handle arguments
JointStateMsgPub::JointStateMsgPub(ros::NodeHandle n, ros::NodeHandle pn)
{
  joint_angle = 0; // pub was intialized in .h file
  pub_joint_state = n.advertise<sensor_msgs::JointState>("joint_states",1); // Advertising sensor_msg
  timer = n.createTimer(ros::Duration(0.02), &JointStateMsgPub::timerCallback, this); // this (instead of function name) = pointer to instance of class
}

void JointStateMsgPub::timerCallback(const ros::TimerEvent& event){
  double constant_speed = 1.0;
  sensor_msgs::JointState joint_state_msg;

  joint_state_msg.position.resize(2); // Could just do push back
  joint_state_msg.name.resize(2);
  joint_state_msg.header.stamp = event.current_real; // Don't forget this
  joint_angle += 0.02*constant_speed;
  joint_state_msg.position[0] = joint_angle;
  joint_state_msg.name[0] = "joint1";
  joint_state_msg.position[1] = joint_angle;
  joint_state_msg.name[1] = "joint2"; // Grab these names from URDF file
  
  pub_joint_state.publish(joint_state_msg);
}


}
