#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "HSVFilter.hpp"

static const std::string OPENCV_WINDOW = "Image Window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher hsv_pub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    hsv_pub_ = it_.advertise("/image_filter/hsv_filtered", 1);
    image_pub_ = it_.advertise("/image_filter/rgb_raw", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    image_pub_.publish(cv_ptr->toImageMsg());
    // HSV filtering based on the input color
    HSVFilter hsvFilter(cv_ptr->image);
    hsvFilter.setColorValues("GREEN");
    cv::Mat thresholdImage = hsvFilter.getFilteredImage();
    hsvFilter.morphOps(thresholdImage);
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, thresholdImage);
    //cv::waitKey(3);

    // Output modified video stream
    cv_ptr->image = thresholdImage;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    hsv_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgb_image_grabber");
  ImageConverter ic;
  ros::spin();
  return 0;
}
