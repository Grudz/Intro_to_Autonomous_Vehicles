// Header file for the class
#include "fundamentals.h"

// Namespace matches ROS package name
namespace basics {

// Constructor with global and private node handle arguments
Fundamentals::Fundamentals(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Define publishers and subscribers here
  timer = n.createTimer(ros::Duration(1), &Fundamentals::timerCallback, this);

  
}
  
void Fundamentals::timerCallback(const ros::TimerEvent& event)
{
    // Print out common data types with ROS INFO
    int _int = 7;
    double _double = 10.7;
    std::string _string = "Hey!";

    ROS_INFO_ONCE("********** PRINTING DATATYPES **********");
    ROS_INFO_ONCE("Print integer: %d\n", _int);
    ROS_INFO_ONCE("Print double: %f\n", _double);
    ROS_INFO_ONCE("Print string: %s\n", _string.c_str());

    // Note use of global counter variable in .h file
    ROS_INFO_ONCE("********** PRINTING ARRAY **********");
    int array [5];

    for (int i = 0; i < counter; i ++)
    {
      array[i] = counter;
      ROS_INFO("array index %d = %d", i, counter); // Regular ROS_INFO here
      counter--;
    }

}

}
