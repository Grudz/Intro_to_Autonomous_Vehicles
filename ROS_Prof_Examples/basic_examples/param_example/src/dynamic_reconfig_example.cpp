#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <param_example/ReconfigExampleConfig.h>

param_example::ReconfigExampleConfig cfg;

void reconfig(param_example::ReconfigExampleConfig& config, uint32_t level){
    ROS_INFO("Current values in parameter server: ");
    
    if(config.enable){
        ROS_INFO("Enabled");
    } else {
        ROS_INFO("Disabled");
    }
    
    if(config.x > 50){
        config.x = 50;
    }
    
    ROS_INFO("X: %f", config.x);
    ROS_INFO("Y: %f", config.y);
    ROS_INFO("String: %s", config.str.c_str());
    
    switch(config.list){
        case 0:
            ROS_INFO("Option 1 Selected");
            break;
        case 1:
            ROS_INFO("Option 2 Selected");
            break;
        case 2:
            ROS_INFO("Option 3 Selected");
            break;
                    
    }
    
    cfg = config;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dynamic_reconfig_example");
    ros::NodeHandle nh;
    
    dynamic_reconfigure::Server<param_example::ReconfigExampleConfig> srv(nh);
    
    srv.setCallback(boost::bind(reconfig, _1, _2));
    
    
    ros::spin();
    
    
}
