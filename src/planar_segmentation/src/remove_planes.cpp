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
  ros::init (argc, argv, "remove_planes");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/downsampled_cloud", 1, cloud_cb_);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl::PCLPointCloud2> ("scene_no_planes", 1);

  ROS_INFO("Waiting for downsampled cloud...");
  // Spin
  ros::spin ();

  return 0;
}

void cloud_cb_ (const pcl::PCLPointCloud2ConstPtr& cloud_msg)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloudPtr cloud(new PointCloudT);
  pcl::PCLPointCloud2 output;

  // PCL PointCloud conversion
  pcl::fromPCLPointCloud2(*cloud_msg, *cloud);
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
  seg.setMaxIterations(100);
  seg.setDistanceThreshold (0.03);

  size_t const original_cloud_size = cloud->size();
  size_t const point_threshold = min_filter_percentage_ * original_cloud_size;
  while(cloud->size() > point_threshold) {
    seg.setInputCloud (cloud);
    // perform segmentation
    seg.segment (*current_plane_indices, coefficients);

    // check if we found any indices in this iteration
    if(current_plane_indices->indices.size() == 0) {
      ROS_WARN("Cannot find more planes");
      break;
    }

    // Remove the currently found inliers from the input cloud (prepare the
    // cloud for the next step)
    extract.setInputCloud(cloud);
    extract.setIndices(current_plane_indices);
    extract.setNegative(true);
    extract.filter(*cloud);
  }

  // convert to pcl message
  pcl::toPCLPointCloud2(*cloud, output);
  // publish the scene with all the possible planes removed
  pub.publish (output);
}
