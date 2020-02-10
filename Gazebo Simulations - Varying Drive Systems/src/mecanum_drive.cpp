// homework3 - Ben Grudzien
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Subscribing to these messages
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher speed_pub;

const double WHEEL_RADIUS_M = 0.15;
const double CAR_WIDTH = 0.5;
const double CAR_LENGTH = 0.5; // Nice square car

void recvTwist(const geometry_msgs::TwistConstPtr &msg){
    
    // Message variables
    double vx = msg->linear.x; // Velocity "direction"
    double vy = msg->linear.y;
    double psi_dot = msg->angular.z;    
    // Parameters
    double a = 1/WHEEL_RADIUS_M;
    double b = a;
    double c = (CAR_LENGTH+CAR_WIDTH)/(2*WHEEL_RADIUS_M);
    // Individual resulting speeds
    double w1 = a*vx-b*vy-c*psi_dot;
    double w2 = a*vx+b*vy+c*psi_dot;
    double w3 = a*vx+b*vy-c*psi_dot;
    double w4 = a*vx-b*vy+c*psi_dot;
   
    std_msgs::Float64MultiArray speed;
    speed.data.resize(4);
    speed.data[0] = w1;
    speed.data[1] = w2;
    speed.data[2] = w3;
    speed.data[3] = w4;
    
    speed_pub.publish(speed);
    
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"mecanum_drive"); // Setting up node
    ros::NodeHandle node;
    
    ros::Subscriber sub_twist = node.subscribe("/twist_cmd", 1, recvTwist); // Subcribing to Twist
    
    speed_pub = node.advertise<std_msgs::Float64MultiArray>("/omnibot/wheel_speed_cmd", 1); // advertising to gazebo

    ros::spin();
}

