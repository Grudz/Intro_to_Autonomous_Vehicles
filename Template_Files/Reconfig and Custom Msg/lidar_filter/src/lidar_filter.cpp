// Create fun gazebo city

// Header file for the class
#include "lidar_filter.h"


// Namespace matches ROS package name
namespace lidar_filter {

  // Constructor with global and private node handle arguments
  BBoxPrep::BBoxPrep(ros::NodeHandle n, ros::NodeHandle pn) :
  kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)  // Intialize in constructor, looking at XYZ, new creates instance and uses pointer
  {
    srv_.setCallback(boost::bind(&BBoxPrep::reconfig, this, _1, _2));  // binds reconfigure to get updated when new params
    pub_passthrough_cloud_= n.advertise<sensor_msgs::PointCloud2>("passthrough_cloud", 1); 
    pub_cluster_cloud_= n.advertise<sensor_msgs::PointCloud2>("cluster_clouds", 1); 
    pub_bbox_= n.advertise<jsk_recognition_msgs::BoundingBoxArray>("bounding_boxes", 1);
    sub_cloud_ = n.subscribe("points", 10, &BBoxPrep::recvCloud, this);

  }  

  void BBoxPrep::reconfig(PointCloudFilterConfig& config, uint32_t level)  
  {
    cfg_ = config;  // cfg_ global in class, declared in hpp
  }

  void BBoxPrep::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    // Create pointer to PCL type variable
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

    // Convert to sensor_msgs::PointCloud2
    pcl::fromROSMsg(*msg, *input_cloud);

    // Put input cloud into passthrough filter to remove ground plane
    passthroughFilter(input_cloud, passthrough_cloud);

    // Create PC cluster filter variable and index variable
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extract_clusters;

    extract_clusters.setClusterTolerance(cfg_.cluster_tol);  // Size of boxes of points to cluster, 0.5m^2 right now
    extract_clusters.setMinClusterSize(cfg_.min_cluster_size); // how many points allowed in every cluster
    extract_clusters.setMaxClusterSize(cfg_.max_cluster_size); // 5 to 500 right now
    kd_tree_->setInputCloud(passthrough_cloud);  // Sorts the passthrough cloud so it's easier to process
    extract_clusters.setSearchMethod(kd_tree_);
    extract_clusters.setInputCloud(passthrough_cloud);
    extract_clusters.extract(cluster_indices);   // Actually perform the extraction

    // Create array of pointcloud pointers. One for each cluster index
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    for (auto cluster_indices : cluster_indices)
    {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>); // Pointcloud pointer to iterate with
      pcl::copyPointCloud(*passthrough_cloud, cluster_indices, *cluster); // input, vector of indices to be copied from, output cloud
      cluster->width = cluster->points.size();  // Fill in fields
      cluster->height = 1;       
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster); 

    }

    // Use max and min of each cluster to create bbox
    pcl::PointXYZ min, max;  // Relative to the Lidar
    ROS_INFO("\n----- Loop Start -----\n");
    ROS_INFO("Clusters in scan = %d\n", (int)cluster_clouds.size());  // Is this so bad? Draw a new box for each new sequence?

    // Copy header from passthrough cloud and clear array
    bbox_array_.header = pcl_conversions::fromPCL(passthrough_cloud->header);
    bbox_array_.boxes.clear();

    // Loop through clusters and box up
    for (auto& cluster : cluster_clouds) 
    {  
      
      // Applying the min/max function
      pcl::getMinMax3D(*cluster, min, max);  // Get min/max 3D
      ROS_INFO("Header seq per cluster = %d\n", (int)cluster->header.seq);

      // Create bbox message, fill in fields, push it into bbox array
      jsk_recognition_msgs::BoundingBox bbox;

      bbox.header = bbox_array_.header;
      bbox.dimensions.x = max.x - min.x;
      bbox.dimensions.y = max.y - min.y;
      bbox.dimensions.z = max.z - min.z;
      bbox.pose.position.x = (max.x + min.x) / 2; 
      bbox.pose.position.y = (max.y + min.y) / 2; 
      bbox.pose.position.z = (max.z + min.z) / 2; 
      bbox.pose.orientation.w = 1.0;
      bbox_array_.boxes.push_back(bbox);

    } 

    pub_bbox_.publish(bbox_array_);

  }

  // Passthrough filter function
  void BBoxPrep::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) 
  {

    // Start filtering
    pcl::IndicesPtr roi_indices(new std::vector <int>); // Will store subset of original array. ROI = region of interest
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter; // Instantiate filter. Will find x, y, z points that will satisfy ROI

    // Put pointer to input cloud in passthrough filter
    passthrough_filter.setInputCloud(cloud_in); // Already wants a pointer so no dereference

    // Index is relative to the Lidar frame
    // Extract X points
    passthrough_filter.setFilterFieldName("x"); // 3 fields, hence PointXYZ
    passthrough_filter.setFilterLimits(cfg_.x_min, cfg_.x_max);
    passthrough_filter.filter(*roi_indices);    // Referes to input cloud

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("y"); 
    passthrough_filter.setFilterLimits(cfg_.y_min, cfg_.y_max);
    passthrough_filter.filter(*roi_indices);    

    // Extract Z points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("z"); 
    passthrough_filter.setFilterLimits(cfg_.z_min, cfg_.z_max);
    passthrough_filter.filter(*cloud_out);     // Final stage so pass filter cloud so instead of int array it's a corresponding point cloud  

    // Downsample the cloud with Voxel Grid filter
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_out);  
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size); // All same so cube
    downsample.filter(*cloud_out);

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 passthrough_cloud_out;
    pcl::toROSMsg(*cloud_out, passthrough_cloud_out);

    // Publish Passthrough filter PointCloud
    pub_passthrough_cloud_.publish(passthrough_cloud_out);

  }

}
