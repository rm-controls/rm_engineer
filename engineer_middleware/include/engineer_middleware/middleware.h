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

namespace engineer_middleware {
class Middleware {
 public:
  explicit Middleware(ros::NodeHandle &nh);
  ros::NodeHandle nh_;
  //action
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  rm_msgs::EngineerFeedback feedback;
  rm_msgs::EngineerResult result_;
 private:
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface hand_group_;
  ChassisInterface chassis_interface_;
  ros::Publisher card_pub, gimbal_pub;
  std::unordered_map<std::string, StepQueue> step_queues_;
  XmlRpc::XmlRpcValue *steps_params_;
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr &goal) {
    std::string step_name;
    step_name = goal->step;

    result_.finish = true;
    action_.setSucceeded(result_);
  }

};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
