#include "ClusterExample.hpp"

namespace ece6460_pointcloud_example
{

  ClusterExample::ClusterExample(ros::NodeHandle& n, ros::NodeHandle& pn) :
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)  // Intialize in constructor, looking at XYZ, new creates instance and uses pointer
  {
    srv_.setCallback(boost::bind(&ClusterExample::reconfig, this, _1, _2));

    // Set up publishers and subscribers
    sub_cloud_ = n.subscribe("points", 10, &ClusterExample::recvCloud, this);
    pub_filtered_cloud_ = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
    pub_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);
  }

  void ClusterExample::reconfig(PointCloudExampleConfig& config, uint32_t level)
  {
    cfg_ = config;
  }

  void ClusterExample::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Instantiate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Copy ROS message data into PCL cloud
    pcl::fromROSMsg(*msg, *input_cloud);

    // Passthrough filter + voxel grid filter
    filterRawCloud(input_cloud, filtered_cloud);  // Filtered cloud is now updated to cloud_out

    // Compute normal vectors for the incoming point cloud, this is computational, so filter first
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);  // Create new cloud but type is normal not XYZ
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;  // Input PC XYZ, output Normal
    kd_tree_->setInputCloud(filtered_cloud);  // Sort filtered cloud with KD tree to correspond with 3D position (sorts array)
    normal_estimator.setSearchMethod(kd_tree_);  // Use this ^^
    normal_estimator.setInputCloud(filtered_cloud);  // Input cloud to process (filtered from previous stage)
    normal_estimator.setKSearch(cfg_.num_normal_neighbors);  // For each point, search for this many nearby ones. It fits them to plane equations then uses cross product to find normal
    normal_estimator.compute(*cloud_normals);  // (Cross product for normals)

    // TODO: Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;  // This is HW3, skelton code here, this filters out ground points
    for (int i = 0; i < cloud_normals->points.size(); i++) {  // Loop over cloud normals
      non_vertical_normals.indices.push_back(i);  // Only push back non-vertical normals
    }

    // Copy non-vertical normals into a separate cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // Create another PC
    pcl::copyPointCloud(*filtered_cloud, non_vertical_normals, *no_ground_cloud);  // (filtered output, non_vertical PC, stored in no ground cloud)

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(filtered_cloud->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = filtered_cloud->points[non_vertical_normals.indices[i]].x;
      p.position.y = filtered_cloud->points[non_vertical_normals.indices[i]].y;
      p.position.z = filtered_cloud->points[non_vertical_normals.indices[i]].z;     

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      // Construct rotation matrix to align frame transform with the normal vector
      tf2::Matrix3x3 rot_mat;
      rot_mat[0] = tf2::Vector3(nx, ny, nz);
      if (std::abs(nz) < 0.9) {
        // Vector is not close to vertical, use x and y components to create orthogonal vector
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
      } else {
        // Vector is close to vertical, use y and z components to make orthogonal vector
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
      }
      // Normalize the generated orthogonal vector, because it is not necessarily unit length
      rot_mat[1].normalize();
      // Cross product produces the third basis vector of the rotation matrix
      rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

      // Extract equivalent quaternion representation for the transform
      // rot_mat.transpose() is used because the basis vectors should be loaded
      // into the columns of the matrix, but the indexing in the above commands set the rows
      //   of the matrix instead of the columns.
      tf2::Quaternion q;
      rot_mat.transpose().getRotation(q);

      // Fill orientation of pose structure
      tf2::convert(q, p.orientation);
      normals_.poses.push_back(p);
    }
    // Publish normal vectors
    pub_normals_.publish(normals_);

    // Run Euclidean clustering and extract set of indices arrays
    std::vector<pcl::PointIndices> cluster_indices;  // Same point indencies as used to store non-vertical normals
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // Object that implaments 
    ec.setClusterTolerance(cfg_.cluster_tol);  // How big boxes should be to cluster boxes, ie: points withen half a meter will be in same group
    ec.setMinClusterSize(cfg_.min_cluster_size);  // How many points allowed in a cluster
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(no_ground_cloud);  // Sort cloud so easier to process
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(no_ground_cloud);
    ec.extract(cluster_indices);  // Applies processing, above is configuration

    // Use indices arrays to separate point cloud into individual clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;  // Create array of point cloud pointers, one for each cluster
    for (auto indices : cluster_indices) {  
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*no_ground_cloud, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }

    // Merge individual cluster clouds into a ROS PointCloud2 message for Rviz debugging
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // Re-merge because not all no_ground_cloud points are clustered
    for (auto& cluster : cluster_clouds) {
      merged_cloud->points.insert(merged_cloud->points.begin(), cluster->points.begin(), cluster->points.end());
    }
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    // Compute bounding boxes around clusters
    pcl::PointXYZ min_point, max_point;

    bboxes_.header = pcl_conversions::fromPCL(filtered_cloud->header);
    bboxes_.objects.clear();
    for (auto& cluster : cluster_clouds) {  // Not using re-merged cloud here
      pcl::getMinMax3D(*cluster, min_point, max_point);  // Get min/max 3D
      avs_lecture_msgs::TrackedObject box;  // rosmsg show TrackedObjectArray
      box.header = bboxes_.header;  // Copy header
      box.bounding_box_scale.x = max_point.x - min_point.x;  // Size of box is difference between min and max form getMinMax3D
      box.bounding_box_scale.y = max_point.y - min_point.y;
      box.bounding_box_scale.z = max_point.z - min_point.z;
      box.pose.position.x = 0.5 * (max_point.x + min_point.x);  // Position reflects center of box, take average of each point
      box.pose.position.y = 0.5 * (max_point.y + min_point.y);
      box.pose.position.z = 0.5 * (max_point.z + min_point.z);
      box.pose.orientation.w = 1.0;  // Don't care about orientation, so set W = 1.0, identity rotation
      bboxes_.objects.push_back(box);
    }
    // Publish bounding boxes
    pub_bboxes_.publish(bboxes_);  // Publishing an entire array

    // Copy filtered cloud data into a ROS message
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*merged_cloud, output_msg);  // copy mergerd cloud to PointCLoud2 msg
    output_msg.header = pcl_conversions::fromPCL(filtered_cloud->header);

    // Publish output point cloud
    pub_filtered_cloud_.publish(output_msg);
  }

  void ClusterExample::filterRawCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud (cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cfg_.x_min, cfg_.x_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (cfg_.y_min, cfg_.y_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (cfg_.z_min, cfg_.z_max);
    pass.filter (*cloud_out);

    // Everything above here is from passthrough example
    // Run through a voxel grid filter to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> downsample;  // voxel is 3D pixel, cube. Defines cube and replaces points in that with 1 point in center
    downsample.setInputCloud(cloud_out);
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size); // same size so cube, this is box size
    downsample.filter(*cloud_out);  // takes leaf size and overwrites cloud_out
  }

}