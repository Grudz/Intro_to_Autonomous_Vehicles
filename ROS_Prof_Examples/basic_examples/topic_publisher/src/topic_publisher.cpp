#include <ros/ros.h>
#include <std_msgs/String.h>

ros::Publisher publisher;

void recvcallback(const std_msgs::StringConstPtr& msg){
    std_msgs::String result;
    result.data = msg->data + "_123";
    publisher.publish(result);
}


int main(int argc, char** argv){
    ros::init(argc,argv,"topic_publisher");
    ros::NodeHandle nodehandle;
    
    ros::Subscriber subscriber = nodehandle.subscribe("/topic_in", 1, recvcallback);
    publisher = nodehandle.advertise<std_msgs::String>("/topic_out", 1);
    
    
    ros::spin();
}
