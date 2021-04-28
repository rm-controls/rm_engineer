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
  arm_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_arm");
  hand_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_hand");
  step_queue_ = new engineer_middleware::StepQueue(steps_params, *arm_group_, *hand_group_);
  steps_params = new XmlRpc::XmlRpcValue;
  action_.start();
}
void Middleware::run() const {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  nh_.getParam("place", *steps_params);
  engineer_middleware::StepQueue step_queue(steps_params, *arm_group_, *hand_group_);
  step_queue.move();
  steps_params->clear();
  nh_.getParam("grasp_groud", *steps_params);
  step_queue.reload(*steps_params);
  step_queue.move();
}
}