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
  steps_params_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("grasp_big_resource", *steps_params_);
  step_queue_ = new engineer_middleware::StepQueue(*steps_params_, *arm_group_, *hand_group_);
  action_.start();
}
void Middleware::chassisThread() {
  ROS_INFO_STREAM("chassis thread id=" << boost::this_thread::get_id());
  ros::NodeHandle n;
}
void Middleware::run() const {
  boost::thread::id main_id = boost::this_thread::get_id();
  std::cout << "main thread id is :" << main_id << std::endl;
  boost::thread chassis_thread(chassisThread);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("start 1 action");
  step_queue_->move();
  ROS_INFO("action 1 finish");
}

}