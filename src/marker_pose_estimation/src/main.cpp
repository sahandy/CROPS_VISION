#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

void pose_cb_ (const geometry_msgs::PoseStamped& pose)
{
  std::cout << "   ===   ===   ===" << std::endl;
  std::cout << "stamp: " << pose.header.stamp << std::endl;
  std::cout << "frame_id: " << pose.header.frame_id << std::endl;
  std::cout << "pose: " << std::endl;
  std::cout << "   position: " << pose.pose.position.x << ", "
                               << pose.pose.position.y << ", "
                               << pose.pose.position.z << ", " << std::endl;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/ar_single_board/pose", 1, pose_cb_);

  // Spin
  ros::spin ();

  return 0;
}
