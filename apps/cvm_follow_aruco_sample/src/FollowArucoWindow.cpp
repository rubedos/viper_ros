/*******************************************************************************
* Copyright (c) 2019 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#include "FollowArucoWindow.h"
#include "ui_FollowArucoWindow.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <qevent.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

using cvm_follow_aruco_sample::FollowArucoWindow;

FollowArucoWindow::FollowArucoWindow(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::FollowArucoWindow())
  , m_imageSub()
  , m_statusSub()
  , m_boundingBoxesSub()
  , m_targetBoundingBoxSub()
  , m_targetPositionSub()
  , m_dynParametersSub()
  , m_enablePub()
  , m_prefix()
{
  ui->setupUi(this);
  ui->display_image->installEventFilter(this);
  enableInfoLabels(false);

  // Connect event handlers
  connect(ui->enableFollowArucoButton, SIGNAL(released()), this, SLOT(enableReleased()));
  connect(ui->markerSizeSlider, SIGNAL(sliderMoved(int)), this, SLOT(markerSizeChanged(int)));
  connect(ui->markerIdSlider, SIGNAL(sliderMoved(int)), this, SLOT(markerIdChanged(int)));
}

FollowArucoWindow::~FollowArucoWindow()
{
  m_imageSub.shutdown();
  m_statusSub.shutdown();
  m_boundingBoxesSub.shutdown();
  m_targetBoundingBoxSub.shutdown();
  m_targetPositionSub.shutdown();
  m_dynParametersSub.shutdown();

  delete ui;
}

void FollowArucoWindow::init(const std::string& prefix)
{
  m_prefix = prefix;
  ros::NodeHandle n;
  // Initialize subscribers
  m_imageSub = n.subscribe<sensor_msgs::Image>(m_prefix + "/follow_aruco/detection_image", 1, &FollowArucoWindow::imageCb, this);
  m_statusSub = n.subscribe<std_msgs::String>(m_prefix + "/follow_aruco/status", 1, &FollowArucoWindow::statusCb, this);
  m_targetPositionSub = n.subscribe<geometry_msgs::PointStamped>(m_prefix + "/follow_aruco/target_position", 1, &FollowArucoWindow::targetPositionCb, this);
  m_dynParametersSub = n.subscribe<dynamic_reconfigure::Config>(m_prefix + "/cvm_follow_aruco/parameter_updates", 1,&FollowArucoWindow::parameterUpdates, this);
  // Initialize publishers
  m_enablePub = n.advertise<std_msgs::Bool>(m_prefix + "/follow_aruco/enable", 1);
}

void FollowArucoWindow::enableReleased()
{
  std_msgs::Bool msg;
  if(ui->enableFollowArucoButton->text().contains("Enable"))
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  m_enablePub.publish(msg);
}

void FollowArucoWindow::markerSizeChanged(int value)
{
  if(value == ui->markerSizeSlider->value())
    return;
  double finalValue = (float)value / 100.0f;
  ui->markerSizeLabel->setText(QString::number(finalValue, 'f', 2) + QString(" m"));
  // Update UI
  update();

  changeDoubleParameter("marker_size", finalValue);
}

void FollowArucoWindow::markerIdChanged(int value)
{
  if(value == ui->markerIdSlider->value())
    return;
  double finalValue = value;
  ui->markerIdLabel->setText(QString::number(finalValue));
  // Update UI
  update();

  changeIntParameter("marker_id", finalValue);
}

void FollowArucoWindow::imageCb(const sensor_msgs::ImageConstPtr& image)
{
  const cv::Mat img = cv_bridge::toCvShare(image)->image;
  //
  // Resize the image to fit the label
  //
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(ui->display_image->width(), ui->display_image->height()));
  QImage imdisplay((uchar*)resized.data, resized.cols, resized.rows, resized.step, QImage::Format_RGB888);
  ui->display_image->setPixmap(QPixmap::fromImage(imdisplay));
}

void FollowArucoWindow::statusCb(const std_msgs::StringConstPtr& status)
{
  ui->statusLabel->setText(QString::fromStdString(std::string(status->data)));
  if(ui->statusLabel->text().contains("disabled")) // If FollowAruco is disabled
  {
    if(!ui->enableFollowArucoButton->text().contains("Enable FollowAruco"))
    {
      ui->enableFollowArucoButton->setText(QString("Enable FollowAruco"));
      enableInfoLabels(false);
    }
  }
  else if(!ui->enableFollowArucoButton->text().contains("Disable FollowAruco"))
  {
    ui->enableFollowArucoButton->setText(QString("Disable FollowAruco"));
    enableInfoLabels(true);
  }
}

void FollowArucoWindow::targetPositionCb(const geometry_msgs::PointStampedConstPtr& point)
{
  ui->xLabel->setText(QString("x: ") + QString::number(point->point.x, 'f', 2));
  ui->yLabel->setText(QString("y: ") + QString::number(point->point.y, 'f', 2));
  ui->zLabel->setText(QString("z: ") + QString::number(point->point.z, 'f', 2));
}

void FollowArucoWindow::enableInfoLabels(bool status)
{
  // Set value only if changes are required
  if(ui->label_2->isVisible() && !status ||
     !ui->label_2->isVisible() && status)
  {
    ui->label_2->setVisible(status);
    ui->xLabel->setVisible(status);
    ui->yLabel->setVisible(status);
    ui->zLabel->setVisible(status);
    // Update UI
    update();
  }
}

void FollowArucoWindow::changeDoubleParameter(const std::string& name, double value)
{
  dynamic_reconfigure::DoubleParameter parameter;
  parameter.name = name;
  parameter.value = value;

  dynamic_reconfigure::Config config;
  config.doubles.push_back(parameter);

  dynamic_reconfigure::ReconfigureRequest serverRequest;
  serverRequest.config = config;

  dynamic_reconfigure::ReconfigureResponse serverResponse;
  ros::service::call(m_prefix + "/cvm_follow_aruco/set_parameters", serverRequest, serverResponse);
}

void FollowArucoWindow::changeIntParameter(const std::string& name, int value)
{
  dynamic_reconfigure::IntParameter parameter;
  parameter.name = name;
  parameter.value = value;

  dynamic_reconfigure::Config config;
  config.ints.push_back(parameter);

  dynamic_reconfigure::ReconfigureRequest serverRequest;
  serverRequest.config = config;

  dynamic_reconfigure::ReconfigureResponse serverResponse;
  ros::service::call(m_prefix + "/cvm_follow_aruco/set_parameters", serverRequest, serverResponse);
}

void FollowArucoWindow::parameterUpdates(const dynamic_reconfigure::ConfigConstPtr& config)
{
  for(auto param : config->doubles)
  {
    if(param.name == "marker_size")
    {
      int value = param.value * 100;
      ui->markerSizeLabel->setText(QString::number(param.value, 'f', 2) + QString(" m"));
      ui->markerSizeSlider->setValue(value);
      // Update UI
      update();
    }
  }
  for(auto param : config->ints)
  {
    if(param.name == "marker_id")
    {
      int value = param.value;
      ui->markerIdLabel->setText(QString::number(param.value));
      ui->markerIdSlider->setValue(value);
      // Update UI
      update();
    }
  }
}
