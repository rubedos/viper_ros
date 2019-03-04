/*******************************************************************************
* Copyright (c) 2019 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#ifndef FOLLOW_ARUCO_WINDOW_H
#define FOLLOW_ARUCO_WINDOW_H

#include <QMainWindow>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/Config.h>
#include <string>

namespace Ui
{
  // Class generated from .ui file
  class FollowArucoWindow;
}  // namespace Ui

namespace cvm_follow_aruco_sample
{

class FollowArucoWindow : public QMainWindow
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit FollowArucoWindow(QWidget *parent = 0);
  /**
   * @brief Destructor
   */
  virtual ~FollowArucoWindow();
  void init(const std::string& prefix = "DEFAULT");
private Q_SLOTS:
  /**
   * @brief Method that is invoked when FollowAruco status button is released
   */
  void enableReleased();
  /**
   * @brief Method that is invoked when marker size value is changed
   * @param value Slider value
   */
  void markerSizeChanged(int value);
  /**
   * @brief Method that is invoked when marker id slider value is changed
   * @param value Slider value
   */
  void markerIdChanged(int value);
private:
  /**
   * @brief Method that is called when detection image from camera is received. This method is respondible
   * for displaying images and bounding boxes.
   * @param image detection image message
   */
  void imageCb(const sensor_msgs::ImageConstPtr& image);
  /**
   * @brief Method that is called when FollowAruco status from camera is received. This method is responsible for
   * state managment of the sample app. Status and UI with it are changed based on the status received.
   * @param status Status message
   */
  void statusCb(const std_msgs::StringConstPtr& status);
  /**
   * @brief Method that is called when position of the target is received
   * @param point Position (x, y, z) of the target
   */
  void targetPositionCb(const geometry_msgs::PointStampedConstPtr& point);
  /**
   * @brief Method that changes visiblity of GUI which displays distance information
   * @param status Status indicating to set visibility on or off
   */
  void enableInfoLabels(bool status);
  /**
   * @brief Method that changes value of given dynamic reconfigure double parameter
   * @param name Name of dynamic reconfigure parameter that has type double
   * @param value Value of the parameter
   */
  void changeDoubleParameter(const std::string& name, double value);
  /**
   * @brief Method that changes value of given dynamic reconfigure int parameter
   * @param name Name of dynamic reconfigure parameter that has type double
   * @param value Value of the parameter
   */
  void changeIntParameter(const std::string& name, int value);
  /**
  /**
   * @brief Method that is called when dynamic reconfigure parameters of FollowAruco are updated
   * @param config Config file containing all parameters
   */
  void parameterUpdates(const dynamic_reconfigure::ConfigConstPtr& config);
  // Pointer to a common object that contains all objects used in the app
  Ui::FollowArucoWindow* ui;
  // Subscribers
  ros::Subscriber m_imageSub;
  ros::Subscriber m_statusSub;
  ros::Subscriber m_boundingBoxesSub;
  ros::Subscriber m_targetBoundingBoxSub;
  ros::Subscriber m_targetPositionSub;
  ros::Subscriber m_dynParametersSub;
  // Publishers
  ros::Publisher m_enablePub;
  std::string m_prefix;
};

}  // namespace cvm_follow_aruco_sample

#endif  // FOLLOW_ARUCO_WINDOW_H
