//
// Created by astro on 2021/4/25.
//

#ifndef SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

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
  //chassis
  Chassis *chassis_;
  void executeCB(const rm_msgs::EngineerGoalConstPtr &goal) {

  }
  void run() const;

};
}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
