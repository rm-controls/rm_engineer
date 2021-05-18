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
  middleware.run();
  ros::shutdown();
  return 0;
}