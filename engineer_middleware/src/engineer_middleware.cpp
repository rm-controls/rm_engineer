//
// Created by qiayuan on 4/4/21.
//
#include <ros/ros.h>

#include "engineer_middleware/middleware.h"
using namespace engineer_middleware;
int main(int argc, char **argv) {
  ros::init(argc, argv, "engineer_middleware");
  ros::NodeHandle nh("~");
  Middleware middleware(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time last = ros::Time::now();
  ros::Rate loop_rate(100);
  middleware.raiseArm();
  while (ros::ok()) {
    middleware.run(ros::Time::now() - last);
    last = ros::Time::now();
    loop_rate.sleep();
  }
  return 0;
}