#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Helper typedefs to make the implementation code cleaner
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudPtr;
typedef typename PointCloudT::ConstPtr CloudConstPtr;

// Global variables and constants
ros::Publisher pub;
double const min_filter_percentage_ = 0.1;


void cloud_cb_ (const pcl::PCLPointCloud2ConstPtr& cloud_msg);

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_largest_plane");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/downsampled_cloud", 1, cloud_cb_);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl::PCLPointCloud2> ("planes_largest", 1);

  ROS_INFO("Waiting for downsampled cloud...");
  // Spin
  ros::spin ();
}

void cloud_cb_ (const pcl::PCLPointCloud2ConstPtr& cloud_msg)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloudT cloud;
  PointCloudPtr largest_plane(new PointCloudT);
  pcl::PCLPointCloud2 output;

  // PCL PointCloud conversion
  pcl::fromPCLPointCloud2(*cloud_msg, cloud);
  /*
   * Extract Planar surfaces
   */
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr current_plane_indices(new pcl::PointIndices);

  // instance that will be used to extract the points of the largest found
  // plane
  pcl::ExtractIndices<PointT>  extract;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional parameter for optimization
  seg.setOptimizeCoefficients (true);
  // Mandatory parameters required by segmentation algorithm
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud.makeShared());
  // perform segmentation
  seg.segment (*current_plane_indices, coefficients);
  // Create a cloud out of the currently found plane indices
  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(current_plane_indices);
  extract.setNegative(false);
  extract.filter(*largest_plane);
  // convert to pcl message
  pcl::toPCLPointCloud2(*largest_plane, output);
  // publish the result
  pub.publish (output);
}
