/*******************************************************************************
* Copyright (c) 2018 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#include "FollowMeWindow.h"
#include "ui_FollowMeWindow.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <qevent.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

using cvm_follow_me_sample::FollowMeWindow;

FollowMeWindow::FollowMeWindow(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::FollowMeWindow())
  , m_imageSub()
  , m_statusSub()
  , m_boundingBoxesSub()
  , m_targetBoundingBoxSub()
  , m_targetPositionSub()
  , m_dynParametersSub()
  , m_enablePub()
  , m_setTargetPub()
  , m_boundingBoxes()
  , m_targetBoundingBox()
  , m_width()
  , m_height()
  , m_showingBoundingBoxes()
  , m_trackingOn()
  , m_prefix()
{
  ui->setupUi(this);
  ui->display_image->installEventFilter(this);
  enableInfoLabels(false);

  // Connect event handlers
  connect(ui->enableFollowMeButton, SIGNAL(released()), this, SLOT(enableReleased()));
  connect(ui->dropTimeSlider, SIGNAL(sliderMoved(int)), this, SLOT(dropTimeChanged(int)));
  connect(ui->detectionProbSlider, SIGNAL(sliderMoved(int)), this, SLOT(detectionProbChanged(int)));
  connect(ui->similarityThSlider, SIGNAL(sliderMoved(int)), this, SLOT(similarityThresholdChanged(int)));
  connect(ui->historyQueSlider, SIGNAL(sliderMoved(int)), this, SLOT(historyQueueChanged(int)));
}

void FollowMeWindow::init(const std::string& prefix)
{
  m_prefix = prefix;
  // Initialize subscribers
  ros::NodeHandle n;
  // Initialize subscribers
  m_imageSub = n.subscribe<sensor_msgs::Image>(m_prefix + "/left/image_rect", 1, &FollowMeWindow::leftimageCb, this);
  m_statusSub = n.subscribe<std_msgs::String>(m_prefix + "/follow_me/status", 1, &FollowMeWindow::statusCb, this);
  m_boundingBoxesSub = n.subscribe<cvm_msgs::BoundingBoxes>(m_prefix + "/follow_me/bounding_boxes", 1, &FollowMeWindow::boundingBoxesCb, this);
  m_targetBoundingBoxSub = n.subscribe<cvm_msgs::BoundingBox>(m_prefix + "/follow_me/target_bounding_box", 1, &FollowMeWindow::targetBoxCb, this);
  m_targetPositionSub = n.subscribe<geometry_msgs::PointStamped>(m_prefix + "/follow_me/target_position", 1, &FollowMeWindow::targetPositionCb, this);
  m_dynParametersSub = n.subscribe<dynamic_reconfigure::Config>(m_prefix + "/cvm_follow_me/parameter_updates", 1,&FollowMeWindow::parameterUpdates, this);
  // Initialize publishers
  m_enablePub = n.advertise<std_msgs::Bool>(m_prefix + "/follow_me/enable", 1);
  m_setTargetPub = n.advertise<cvm_msgs::BoundingBox>(m_prefix + "/follow_me/set_target", 1);
}

FollowMeWindow::~FollowMeWindow()
{
  m_imageSub.shutdown();
  m_statusSub.shutdown();
  m_boundingBoxesSub.shutdown();
  m_targetBoundingBoxSub.shutdown();
  m_targetPositionSub.shutdown();
  m_dynParametersSub.shutdown();

  delete ui;
}

bool FollowMeWindow::eventFilter(QObject* watched, QEvent* event)
{
  if(watched != ui->display_image)
    return false;
  if(event->type() != QEvent::MouseButtonPress)
    return false;
  //
  // Find the bounding box that has been clicked and send it to FollowMe as a target
  //
  if(m_showingBoundingBoxes && m_boundingBoxes)
  {
    const QMouseEvent* const mouseEvent = static_cast<const QMouseEvent*>(event);
    const QPoint point = mouseEvent->pos();
    // Remap coordinates
    int x = (int)((float)point.x() / ui->display_image->width() * m_width);
    int y = (int)((float)point.y() / ui->display_image->height() * m_height);
    for(cvm_msgs::BoundingBox box : m_boundingBoxes->boundingBoxes)
    {
      if(x > box.xmin && x < box.xmax &&
         y > box.ymin && y < box.ymax)
      {
        // If user has clicked inside set target
        m_setTargetPub.publish(box);
        return true;
      }
    }
  }
  return true;
}

void FollowMeWindow::enableReleased()
{
  std_msgs::Bool msg;
  if(ui->enableFollowMeButton->text().contains("Enable"))
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  m_enablePub.publish(msg);
}

void FollowMeWindow::dropTimeChanged(int value)
{
  if(value == ui->dropTimeSlider->value())
    return;
  double finalValue = (float)value / 10.0f;
  ui->dropTimeLabel->setText(QString::number(finalValue, 'f', 1) + QString(" s"));
  // Update UI
  update();

  changeDoubleParameter("drop_time", finalValue);
}

void FollowMeWindow::detectionProbChanged(int value)
{
  if(value == ui->detectionProbSlider->value())
    return;
  double finalValue = (float)value / 100.0f;
  ui->detectionProbLabel->setText(QString::number(finalValue, 'f', 2));
  // Update UI
  update();

  changeDoubleParameter("detection_prob", finalValue);
}

void FollowMeWindow::similarityThresholdChanged(int value)
{
  if(value == ui->similarityThSlider->value())
    return;
  double finalValue = (float)value / 100.0f;
  ui->similarityThLabel->setText(QString::number(finalValue, 'f', 2));
  // Update UI
  update();

  changeDoubleParameter("similarity_threshold", finalValue);
}

void FollowMeWindow::historyQueueChanged(int value)
{
  if(value == ui->historyQueSlider->value())
    return;
  double finalValue = value;
  ui->historyQueLabel->setText(QString::number(finalValue));
  // Update UI
  update();

  changeIntParameter("history_queue", finalValue);
}



void FollowMeWindow::leftimageCb(const sensor_msgs::ImageConstPtr& image)
{
  const cv::Mat img = cv_bridge::toCvShare(image)->image;
  cv::Mat result = img.clone();
  m_width = image->width;
  m_height = image->height;
  //
  // Draw bounding boxes of all detected humans in green
  //
  if(m_boundingBoxes)
  {
    if(std::abs(m_boundingBoxes->header.stamp.toSec() - image->header.stamp.toSec()) < 0.5)
    {
      for(cvm_msgs::BoundingBox box : m_boundingBoxes->boundingBoxes)
      {
        cv::Rect2d roi(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);
        cv::rectangle(result, roi, cv::Scalar(0, 255, 0), 2, 1);
      }
      m_showingBoundingBoxes = true;
    }
    else
    {
      m_showingBoundingBoxes = false;
    }
  }
  //
  // Draw the bounding box of human being tracked in red
  //
  if(m_trackingOn && m_targetBoundingBox)
  {
    if(std::abs(m_targetBoundingBox->header.stamp.toSec() - image->header.stamp.toSec()) < 0.5)
    {
      cv::Rect2d roi(m_targetBoundingBox->xmin, m_targetBoundingBox->ymin, m_targetBoundingBox->xmax - m_targetBoundingBox->xmin, m_targetBoundingBox->ymax - m_targetBoundingBox->ymin);
      cv::rectangle(result, roi, cv::Scalar(255, 0, 0), 2, 1);
      m_showingBoundingBoxes = true;
    }
  }
  //
  // Resize the image to fit the label
  //
  cv::resize(result, result, cv::Size(ui->display_image->width(), ui->display_image->height()));
  QImage imdisplay((uchar*)result.data, result.cols, result.rows, result.step, QImage::Format_RGB888);
  ui->display_image->setPixmap(QPixmap::fromImage(imdisplay));
}

void FollowMeWindow::statusCb(const std_msgs::StringConstPtr& status)
{
  ui->statusLabel->setText(QString::fromStdString(std::string(status->data)));
  if(ui->statusLabel->text().contains("disabled")) // If FollowMe is disabled
  {
    if(!ui->enableFollowMeButton->text().contains("Enable FollowMe"))
      ui->enableFollowMeButton->setText(QString("Enable FollowMe"));
    m_trackingOn = false;
  }
  else // FollowMe is enabled
  {
    if(!ui->enableFollowMeButton->text().contains("Disable FollowMe"))
      ui->enableFollowMeButton->setText(QString("Disable FollowMe"));
    if(ui->statusLabel->text().contains("Waiting for another target")) // Enabled and waiting for target to track
    {
      m_trackingOn = false;
    }
    else if(ui->statusLabel->text().contains("Tracking target")) // Target is being tracked
    {
      m_trackingOn = true;
    }
  }
  //
  // Change visibility of info labels based on tracking status
  //
  enableInfoLabels(m_trackingOn);
}

void FollowMeWindow::boundingBoxesCb(const cvm_msgs::BoundingBoxesConstPtr& boundingBoxes)
{
  m_boundingBoxes = boundingBoxes;
}

void FollowMeWindow::targetBoxCb(const cvm_msgs::BoundingBoxConstPtr& bounding_box)
{
  m_targetBoundingBox = bounding_box;
}

void FollowMeWindow::targetPositionCb(const geometry_msgs::PointStampedConstPtr& point)
{
  ui->xLabel->setText(QString("x: ") + QString::number(point->point.x, 'f', 2));
  ui->yLabel->setText(QString("y: ") + QString::number(point->point.y, 'f', 2));
  ui->zLabel->setText(QString("z: ") + QString::number(point->point.z, 'f', 2));
}

void FollowMeWindow::enableInfoLabels(bool status)
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

void FollowMeWindow::changeDoubleParameter(const std::string& name, double value)
{
  dynamic_reconfigure::DoubleParameter parameter;
  parameter.name = name;
  parameter.value = value;

  dynamic_reconfigure::Config config;
  config.doubles.push_back(parameter);

  dynamic_reconfigure::ReconfigureRequest serverRequest;
  serverRequest.config = config;

  dynamic_reconfigure::ReconfigureResponse serverResponse;
  ros::service::call(m_prefix + "/cvm_follow_me/set_parameters", serverRequest, serverResponse);
}

void FollowMeWindow::changeIntParameter(const std::string& name, int value)
{
  dynamic_reconfigure::IntParameter parameter;
  parameter.name = name;
  parameter.value = value;

  dynamic_reconfigure::Config config;
  config.ints.push_back(parameter);

  dynamic_reconfigure::ReconfigureRequest serverRequest;
  serverRequest.config = config;

  dynamic_reconfigure::ReconfigureResponse serverResponse;
  ros::service::call(m_prefix + "/cvm_follow_me/set_parameters", serverRequest, serverResponse);
}

void FollowMeWindow::parameterUpdates(const dynamic_reconfigure::ConfigConstPtr& config)
{
  for(auto param : config->doubles)
  {
    if(param.name == "drop_time")
    {
      int value = param.value * 10;
      ui->dropTimeLabel->setText(QString::number(param.value, 'f', 1) + QString(" s"));
      ui->dropTimeSlider->setValue(value);
      // Update UI
      update();
    }
    else if(param.name == "detection_prob")
    {
      int value = param.value * 100;
      ui->detectionProbLabel->setText(QString::number(param.value, 'f', 2));
      ui->detectionProbSlider->setValue(value);
      // Update UI
      update();
    }
    else if(param.name == "similarity_threshold")
    {
      int value = param.value * 100;
      ui->similarityThLabel->setText(QString::number(param.value, 'f', 2));
      ui->similarityThSlider->setValue(value);
      // Update UI
      update();
    }
  }
  for(auto param : config->ints)
  {
    if(param.name == "history_queue")
      {
        int value = param.value;
        ui->historyQueLabel->setText(QString::number(param.value));
        ui->historyQueSlider->setValue(value);
        // Update UI
        update();
      }
  }
}
