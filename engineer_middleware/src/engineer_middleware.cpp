//
// Created by qiayuan on 4/4/21.
//
#include "engineer_middleware/step_queue.h"

#include <ros/ros.h>

const double tau = 2 * M_PI;

int main(int argc, char **argv) {
  ros::init(argc, argv, "engineer_middleware");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("engineer_arm");
  XmlRpc::XmlRpcValue steps_params;
  nh.getParam("steps", steps_params);
  engineer_middleware::StepQueue step_queue(steps_params, arm_group);
  step_queue.move();

  ros::shutdown();
  return 0;
}