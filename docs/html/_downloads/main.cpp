/*******************************************************************************
* Copyright (c) 2018 by Rubedos
* Kaunas, Lithuania, www.rubedos.com
* All rights reserved.
*******************************************************************************/

#include "FollowMeWindow.h"
#include <ros/spinner.h>
#include <QApplication>

using cvm_follow_me_sample::FollowMeWindow;

int main(int argc, char *argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "FollowMeSample");

  // Start a spinner with 4 threads
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Init QT application
  QApplication a(argc, argv);
  FollowMeWindow w;
  w.show();

  return a.exec();
}
