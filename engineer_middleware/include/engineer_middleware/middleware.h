//
// Created by astro on 2021/4/25.
//

#ifndef SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

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

  BaseController *base_motion_;
  std::thread *base_motion_thread_;
  //server client
  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController srv_;
  //action
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  rm_msgs::EngineerFeedback freedback_;
  rm_msgs::EngineerResult result_;
  //arm
  moveit::planning_interface::MoveGroupInterface *arm_group_;
  moveit::planning_interface::MoveGroupInterface *hand_group_;
  engineer_middleware::StepQueue *step_queue_;
  XmlRpc::XmlRpcValue *steps_params_;
  void executeCB(const rm_msgs::EngineerGoalConstPtr &goal) {

  }
  void run() const;
  void switchController(const std::string &start_controller, const std::string &stop_controller);
};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
