//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware {
Middleware::Middleware(ros::NodeHandle &nh) : nh_(nh), action_(nh_,
                                                               "move_arm",
                                                               boost::bind(&Middleware::executeCB, this, _1),
                                                               false) {
  arm_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_arm");
  hand_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_hand");
  vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  gimbal_cmd_pub_ = nh.advertise<rm_msgs::GimbalCmd>("/cmd_gimbal", 1);
  steps_params_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("grasp_small_resource", *steps_params_);
  step_queue_ =
      new engineer_middleware::StepQueue(*steps_params_, tf_, *arm_group_, *hand_group_, vel_cmd_pub_, gimbal_cmd_pub_);
  switch_controller_client_ =
      nh_.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  action_.start();

}
void Middleware::switchController(const std::string &start_controller, const std::string &stop_controller) {
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers.push_back(start_controller);
  srv.request.stop_controllers.push_back(stop_controller);
  srv.request.strictness = srv.request.BEST_EFFORT;
  srv.request.start_asap = true;
  srv.request.timeout = 0.1;
  if (switch_controller_client_.call(srv))
    ROS_INFO("switch %s to %s", stop_controller.c_str(), start_controller.c_str());
  else
    ROS_INFO("can not switch controller");
}

}