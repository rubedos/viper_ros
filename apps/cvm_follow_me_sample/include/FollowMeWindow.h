/*******************************************************************************
* Copyright (c) 2018 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#ifndef FOLLOW_ME_WINDOW_H
#define FOLLOW_ME_WINDOW_H

#include <QMainWindow>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cvm_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/Config.h>

namespace Ui
{
  // Class generated from .ui file
  class FollowMeWindow;
}  // namespace Ui

namespace cvm_follow_me_sample
{

class FollowMeWindow : public QMainWindow
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit FollowMeWindow(QWidget *parent = 0);
  /**
   * @brief Destructor
   */
  virtual ~FollowMeWindow();
private Q_SLOTS:
  /**
   * @brief Method that is invoked when an even occurs in the given QObject
   * @param watched Object that is being watched by the filter
   * @param event Even that has occurred
   * @return Event handling status
   */
  bool eventFilter(QObject* watched, QEvent* event);
  /**
   * @brief Method that is invoked when FollowMe status button is released
   */
  void enableReleased();
  /**
   * @brief Method that is invoked when drop time slider value is changed
   * @param value Slider value
   */
  void dropTimeChanged(int value);
  /**
   * @brief Method that is invoked when detection probability slider value is changed
   * @param value Slider value
   */
  void detectionProbChanged(int value);
  /**
   * @brief Method that is invoked when similarity threshold slider value is changed
   * @param value Slider value
   */
  void similarityThresholdChanged(int value);
  /**
   * @brief Method that is invoked when history queue slider value is changed
   * @param value Slider value
   */
  void historyQueueChanged(int value);
private:
  /**
   * @brief Method that is called when left image from camera is received. This method is respondible
   * for displaying images and bounding boxes.
   * @param image Left image message
   */
  void leftimageCb(const sensor_msgs::ImageConstPtr& image);
  /**
   * @brief Method that is called when FollowMe status from camera is received. This method is responsible for
   * state managment of the sample app. Status and UI with it are changed based on the status received.
   * @param status Status message
   */
  void statusCb(const std_msgs::StringConstPtr& status);
  /**
   * @brief Method that is called when bounding boxes of humans detected by FollowMe are received
   * @param boundingBoxes Vector of bounding boxes
   */
  void boundingBoxesCb(const cvm_msgs::BoundingBoxesConstPtr& boundingBoxes);
  /**
   * @brief Method that is called when bounding box of target is received
   * @param boundingBox Bounding box of target
   */
  void targetBoxCb(const cvm_msgs::BoundingBoxConstPtr& boundingBox);
  /**
   * @brief Method that is called when position of the target is received
   * @param point Position (x, y, z) of the target
   */
  void targetPositionCb(const geometry_msgs::PointConstPtr& point);
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
   * @brief Method that is called when dynamic reconfigure parameters of FollowMe are updated
   * @param config Config file containing all parameters
   */
  void parameterUpdates(const dynamic_reconfigure::ConfigConstPtr& config);
  // Pointer to a common object that contains all objects used in the app
  Ui::FollowMeWindow* ui;
  // Subscribers
  ros::Subscriber m_imageSub;
  ros::Subscriber m_statusSub;
  ros::Subscriber m_boundingBoxesSub;
  ros::Subscriber m_targetBoundingBoxSub;
  ros::Subscriber m_targetPositionSub;
  ros::Subscriber m_dynParametersSub;
  // Publishers
  ros::Publisher m_enablePub;
  ros::Publisher m_setTargetPub;
  // Cache of bounding boxes
  cvm_msgs::BoundingBoxesConstPtr m_boundingBoxes;
  cvm_msgs::BoundingBoxConstPtr m_targetBoundingBox;
  // Image width and height
  int m_width;
  int m_height;
  // Members indicating status
  bool m_showingBoundingBoxes;
  bool m_trackingOn;
};

}  // namespace cvm_follow_me_sample

#endif  // FOLLOW_ME_WINDOW_H
