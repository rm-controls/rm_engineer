//
// Created by astro on 2021/4/25.
//

#ifndef ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_msgs/EngineerAction.h>
#include "engineer_middleware/step_queue.h"
#include "engineer_middleware/base_motion.h"
namespace engineer_middleware {
class Middleware {
 public:
  explicit Middleware(ros::NodeHandle &nh);
  ros::NodeHandle nh_;
  //action
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  rm_msgs::EngineerFeedback feedback;
  rm_msgs::EngineerResult result_;
  void switchController(const std::string &start_controller, const std::string &stop_controller);
 private:
  BaseMotion *base_motion_;
  std::thread *base_motion_thread_;
  //server client
  ros::ServiceClient switch_controller_client_;

  //arm
  moveit::planning_interface::MoveGroupInterface *arm_group_;
  moveit::planning_interface::MoveGroupInterface *hand_group_;
  engineer_middleware::StepQueue *step_queue_;
  XmlRpc::XmlRpcValue *steps_params_;
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr &goal) {
    std::string step_name;
    step_name = goal->step;
    //TODO switch controller
    nh_.getParam(step_name, *steps_params_);
    ROS_INFO("set goal action:  %s ", step_name.c_str());
    step_queue_->reload(*steps_params_);
    base_motion_->setMiddlewareControl(true);
    ROS_INFO("start %s action", step_name.c_str());
    step_queue_->move();
    ROS_INFO("action %s finish", step_name.c_str());
    base_motion_->setMiddlewareControl(false);
    //TODO switch controller
    result_.finish = true;
    action_.setSucceeded(result_);
  }

};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
