// homework3 - Ben Grudzien
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Subscribing to these messages
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher steering_pub;
ros::Publisher speed_pub;

double LENGTH_A = 2.65;
double STEERING_RATIO = 17.3;

void recvTwist(const geometry_msgs::TwistConstPtr &msg){
    
    double v = msg->linear.x;
    double psi_dot = msg->angular.z;

    std_msgs::Float64 steering_angle;
    geometry_msgs::Twist speed;
    
    speed.linear.x = v;
    steering_angle.data = STEERING_RATIO*atan((LENGTH_A*psi_dot)/v);

    speed_pub.publish(speed);
    steering_pub.publish(steering_angle);
    
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"ackermann");
    ros::NodeHandle node;
    
    ros::Subscriber sub_twist = node.subscribe("/twist_cmd", 1, recvTwist); // Subcribing to Twist
    
    steering_pub = node.advertise<std_msgs::Float64>("/audibot/steering_cmd", 1);
    speed_pub = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1);
    
    ros::spin();
}

