#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_example1");
    ros::NodeHandle nh;
    
    Eigen::Vector3d T_VG; // Transpose vector basically
    Eigen::Matrix3d R_VG;
    
    
    T_VG << 10, 3, 0; // note carrot keys
    R_VG << cos(3.14*2/3), -sin(3.14*2/3), 0, sin(3.14*2/3), cos(3.14*2/3), 0, 0, 0, 1;
    
    ROS_INFO_STREAM("\n" << R_VG); // like cout
    ROS_INFO_STREAM("\n" << T_VG);
    
    
    Eigen::Vector3d target_vehicle;
    Eigen::Vector3d target_global;
    
    target_vehicle << 25, 0, 0;
    
    target_global = R_VG*target_vehicle + T_VG;
    
    ROS_INFO_STREAM("\n" << target_global);    
}
