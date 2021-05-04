//
// Created by astro on 2021/4/25.
//

#ifndef SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>

#include <rm_msgs/EngineerAction.h>

#include "engineer_middleware/step_queue.h"
#include "engineer_middleware/chassis.h"
namespace engineer_middleware {
class Middleware {
 public:
  Middleware(ros::NodeHandle &nh);
  ros::NodeHandle nh_;
  //action
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  rm_msgs::EngineerFeedback freedback_;
  rm_msgs::EngineerResult result_;
  //arm
  moveit::planning_interface::MoveGroupInterface *arm_group_;
  moveit::planning_interface::MoveGroupInterface *hand_group_;
  engineer_middleware::StepQueue *step_queue_;
  XmlRpc::XmlRpcValue *steps_params_;
  //chassis
  void executeCB(const rm_msgs::EngineerGoalConstPtr &goal) {

  }
  void setChassisPosition(double x, double y);
  void disableMiddlewareControl();
  void enableMiddlewareControl();
  void run();

};
struct PipeData {
  double x;
  double y;
};
struct Pipe {
  bool spin_lock_ = 0;
  bool update_ = 0;
  bool middleware_control_ = 0;
  PipeData data_{};
};
}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
