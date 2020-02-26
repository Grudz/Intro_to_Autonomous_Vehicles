#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"time_example");
    ros::NodeHandle nh;
    
    ros::Time current_time = ros::Time::now();
    ros::Duration one_minute(60);
    ros::Time sixty_seconds_later = current_time + one_minute;
    
    ROS_INFO("now: %f       60 seconds later: %f", current_time.toSec(), sixty_seconds_later.toSec());
    ROS_INFO("Difference in time: %f", (sixty_seconds_later - current_time).toSec());
    ros::spin();
}
