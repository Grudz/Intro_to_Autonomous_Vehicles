
#include <ros/ros.h>
#include <service_example/Adder.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "service_client");
    ros::NodeHandle nh;
    
    ros::ServiceClient adder_srv_client = nh.serviceClient<service_example::Adder>("/adder_service");
    
    service_example::AdderRequest req;
    service_example::AdderResponse res;
    
    req.input1 = 5;
    req.input2 = 10;
    
    adder_srv_client.waitForExistence();
    
    bool success = adder_srv_client.call(req,res);
    
    if(success){
        ROS_INFO("The result from the service is: %f", res.result);
    } else {
        ROS_WARN("Your service actually failed to run");
    }
    return 0;
    
    
}
