#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  size_t size = msg->ranges.size();
  ROS_INFO("Distance at %1.2f rads angle is %4.3f meters.", 
           msg->angle_min, msg->ranges[0]);
  ROS_INFO("Distance at %1.2f rads angle is %4.3f meters.", 
           (msg->angle_min + msg->angle_max) / 2.0f, msg->ranges[size / 2]);
  ROS_INFO("Distance at %1.2f rads angle is %4.3f meters.",
            msg->angle_max, msg->ranges[size - 1]); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_streaming");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("laser_scan", 1, laserScanCallback);
  ros::spin();
}
