//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware {
Middleware::Middleware(ros::NodeHandle &nh) : nh_(nh),
                                              action_(nh_,
                                                      "Engineer",
                                                      boost::bind(&Middleware::executeCB, this, _1),
                                                      false) {
  arm_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_arm");
  hand_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_hand");
  steps_params_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("grasp_big_resource", *steps_params_);
  step_queue_ = new engineer_middleware::StepQueue(*steps_params_, *arm_group_, *hand_group_);
  switch_controller_client_ =
      nh_.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  action_.start();
  base_motion_ = new BaseController(nh);
  base_motion_thread_ = new std::thread(&BaseController::baseControllerThread, base_motion_);
}
void Middleware::switchController(const std::string &start_controller, const std::string &stop_controller) {
  srv_.request.start_controllers.push_back(start_controller);
  srv_.request.stop_controllers.push_back(stop_controller);
  srv_.request.strictness = 0;
  srv_.request.start_asap = false;
  srv_.request.timeout = 0.0;
  if (switch_controller_client_.call(srv_))
    ROS_INFO("switch %s to %s", stop_controller.c_str(), start_controller.c_str());
  else
    ROS_INFO("can not switch controller");
}
void Middleware::run() const {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  BaseMotionDate send{};
  send.MiddlewareControl = true;
  send.y = 2.0;
  send.x = 2.0;
  base_motion_->sendDataToBaseController(send);
  ROS_INFO("start 1 action");
  step_queue_->move();
  ROS_INFO("action 1 finish");
  while (ros::ok()) {

  }
}

}