//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
namespace engineer_middleware {

Middleware::Middleware(ros::NodeHandle &nh) : nh_(nh),
                                              action_(nh_,
                                                      "Engineer",
                                                      boost::bind(&Middleware::executeCB, this, _1),
                                                      false) {
  chassis_ = new Chassis(nh);
  action_.start();
}
void Middleware::run() const {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("engineer_arm");
  moveit::planning_interface::MoveGroupInterface hand_group("engineer_hand");
  XmlRpc::XmlRpcValue steps_params;
  nh_.getParam("place", steps_params);
  engineer_middleware::StepQueue step_queue(steps_params, arm_group, hand_group);
  step_queue.move();
  steps_params.clear();
}
}