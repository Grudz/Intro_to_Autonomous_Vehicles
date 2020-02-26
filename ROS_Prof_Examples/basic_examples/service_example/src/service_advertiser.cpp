#include <ros/ros.h>
#include <service_example/Adder.h>

bool srvcallback(service_example::Adder::Request& req,
                 service_example::Adder::Response& res){
    
    res.result = req.input1 + req.input2;
    return true;
}


int main(int argc, char** argv){
    ros::init(argc,argv,"service_advertiser");
    ros::NodeHandle nh;
    
    ros::ServiceServer srv = nh.advertiseService("/adder_service",srvcallback);
    ros::spin();
}
