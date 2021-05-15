//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware {
Pipe pipe;
void chassisThread() {
  ros::NodeHandle nh("~");
  Chassis chassis(nh);
  bool middleware_control = false;
  ROS_INFO_STREAM("chassis start,thread id=" << boost::this_thread::get_id());
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (!pipe.spin_lock_) {
      if (pipe.update_) {
        chassis.setExpectPosition(pipe.data_.x, pipe.data_.y);
        pipe.update_ = false;
      }
      if (pipe.middleware_control_) {
        middleware_control = true;
      } else {
        middleware_control = false;
      }
      if (middleware_control) {
        chassis.move();
      }
    }
    loop_rate.sleep();
  }
}
Middleware::Middleware(ros::NodeHandle &nh) : nh_(nh),
                                              action_(nh_,
                                                      "Engineer",
                                                      boost::bind(&Middleware::executeCB, this, _1),
                                                      false) {
  arm_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_arm");
  hand_group_ = new moveit::planning_interface::MoveGroupInterface("engineer_hand");
  steps_params_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("grasp_groud", *steps_params_);
  step_queue_ = new engineer_middleware::StepQueue(*steps_params_, *arm_group_, *hand_group_);
  switch_controller_client_ =
      nh_.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  action_.start();
}
void Middleware::setChassisPosition(double x, double y) {
  if (!pipe.spin_lock_) {
    pipe.spin_lock_ = true;
    pipe.data_.x = x;
    pipe.data_.y = y;
    pipe.update_ = true;
    pipe.spin_lock_ = false;
  }
}
void Middleware::disableMiddlewareControl() {
  if (!pipe.spin_lock_) {
    pipe.spin_lock_ = true;
    pipe.update_ = true;
    pipe.middleware_control_ = false;
    pipe.spin_lock_ = false;
  }
}
void Middleware::enableMiddlewareControl() {
  if (!pipe.spin_lock_) {
    pipe.spin_lock_ = true;
    pipe.update_ = true;
    pipe.middleware_control_ = true;
    pipe.spin_lock_ = false;
  }
}
void Middleware::switchController() {
  srv_.request.start_controllers.push_back("joint_group_position_controller");
  srv_.request.stop_controllers.push_back("engineer_arm_controller");
  srv_.request.strictness = 0;
  srv_.request.start_asap = false;
  srv_.request.timeout = 0.0;
  if (switch_controller_client_.call(srv_))
    ROS_INFO("switch controller finish");
  else
    ROS_INFO("can not switch controller");
}
void Middleware::run() {
  boost::thread::id main_id = boost::this_thread::get_id();
  std::cout << "main thread id is :" << main_id << std::endl;
  boost::thread chassis_thread(chassisThread);
  //switchController();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("start 1 action");
  enableMiddlewareControl();
  setChassisPosition(2.0, 2.0);
  step_queue_->move();
  ROS_INFO("action 1 finish");

  while (ros::ok()) {

  }
}

}