#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>

void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& msg)
{
  // Extracting disparity image
  const sensor_msgs::Image& dimage = msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  cv::Mat disp_u(dmat.size(), 0);
  // Converts disparity to uint8
  dmat.convertTo(disp_u, disp_u.type());
  cv::imshow("Disparity view", disp_u);
  cv::waitKey(30);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity_streaming");
  ros::NodeHandle nh;
  cv::namedWindow("Disparity view");
  cv::startWindowThread();
  ros::Subscriber sub = nh.subscribe("disparity", 1, disparityImageCallback);
  ros::spin();
  cv::destroyWindow("Disparity view");
}
