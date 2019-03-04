/*******************************************************************************
* Copyright (c) 2019 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#include "FollowArucoWindow.h"
#include <ros/spinner.h>
#include <QApplication>

using cvm_follow_aruco_sample::FollowArucoWindow;

int main(int argc, char *argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "FollowArucoSample");

  // Start a spinner with 4 threads
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Init QT application
  QApplication a(argc, argv);
  FollowArucoWindow w;
  w.init("DEFAULT");  // VIPER prefix
  w.show();

  return a.exec();
}
