#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointType;

class PointCloudProcessor
{
public:
    PointCloudProcessor()
        : m_viewer()
        , m_buffer()
        , m_mutex()
        , m_cloudSubscriber()
    {
    }

  void setViewer(boost::shared_ptr<pcl::visualization::CloudViewer>& viewer)
  {
    m_viewer = viewer;
  }

  void subscribeTo(const std::string& topicName)
  {
    ros::NodeHandle nh;
    m_cloudSubscriber = nh.subscribe<sensor_msgs::PointCloud2>(topicName, 1, boost::bind(&PointCloudProcessor::cloudCallback, this, _1));
  }
private:

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      pcl::PointCloud<PointType>::Ptr pclCloud(new pcl::PointCloud<PointType>);
      pcl::fromROSMsg(*msg, *pclCloud);

      pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());

      pcl::PassThrough<PointType> pass;
      pass.setInputCloud(pclCloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 30);  // (min,max)
      pass.filter(*filtered);

      m_viewer->showCloud(filtered);
    }

    boost::shared_ptr<pcl::visualization::CloudViewer> m_viewer;
    pcl::PointCloud<PointType>::ConstPtr m_buffer;
    boost::mutex m_mutex;
    ros::Subscriber m_cloudSubscriber;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_streaming");
  PointCloudProcessor v;

  boost::shared_ptr<pcl::visualization::CloudViewer> viewer(new pcl::visualization::CloudViewer("PointCloud view"));
  v.setViewer(viewer);
  v.subscribeTo("points2");

  ros::spin();
  return 0;
}
