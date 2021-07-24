#ifndef ENGINEER_MIDDLEWARE_STEP_H_
#define ENGINEER_MIDDLEWARE_STEP_H_

#pragma once

#include "engineer_middleware/motion.h"
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

namespace engineer_middleware {

class Step {
 public:
  Step(const XmlRpc::XmlRpcValue &step, tf2_ros::Buffer &tf,
       moveit::planning_interface::MoveGroupInterface &arm_group, ChassisInterface &chassis_interface,
       ros::Publisher &hand_pub, ros::Publisher &card_pub, ros::Publisher &gimbal_pub) {
    ROS_ASSERT(step.hasMember("step"));
    step_name_ = static_cast<std::string>(step["step"]);
    if (step.hasMember("arm")) {
      if (step["arm"].hasMember("joints"))
        arm_motion_ = new JointMotion(step["arm"], arm_group);
      else
        arm_motion_ = new EndEffectorMotion(step["arm"], arm_group, tf);
    }
    if (step.hasMember("chassis"))
      chassis_motion_ = new ChassisMotion(step["chassis"], chassis_interface);
    if (step.hasMember("hand"))
      hand_motion_ = new HandMotion(step["hand"], hand_pub);
    if (step.hasMember("card"))
      card_motion_ = new JointPositionMotion(step["card"], card_pub);
    if (step.hasMember("gimbal"))
      gimbal_motion_ = new GimbalMotion(step["gimbal"], gimbal_pub);
  }
  bool move() {
    bool success = true;
    if (arm_motion_) success &= arm_motion_->move();
    if (hand_motion_) success &= hand_motion_->move();
    if (card_motion_) success &= card_motion_->move();
    if (chassis_motion_) success &= chassis_motion_->move();
    if (gimbal_motion_) success &= gimbal_motion_->move();
    return success;
  }
  void stop() {
    if (arm_motion_) arm_motion_->stop();
    if (hand_motion_) hand_motion_->stop();
    if (chassis_motion_) chassis_motion_->stop();
  }
  bool isFinish() {
    bool success = true;
    if (arm_motion_) success &= arm_motion_->isFinish();
    if (hand_motion_) success &= hand_motion_->isFinish();
    if (card_motion_) success &= card_motion_->isFinish();
    if (chassis_motion_) success &= chassis_motion_->isFinish();
    if (gimbal_motion_) success &= gimbal_motion_->isFinish();
    return success;
  }
  bool checkTimeout(ros::Duration period) {
    bool success = true;
    if (arm_motion_) success &= arm_motion_->checkTimeout(period);
    if (hand_motion_) success &= hand_motion_->checkTimeout(period);
    if (card_motion_) success &= card_motion_->checkTimeout(period);
    if (chassis_motion_) success &= chassis_motion_->checkTimeout(period);
    if (gimbal_motion_) success &= gimbal_motion_->checkTimeout(period);
    return success;
  }
  std::string getName() { return step_name_; }
 private:
  std::string step_name_;
  MoveitMotionBase *arm_motion_{};
  HandMotion *hand_motion_{};
  JointPositionMotion *card_motion_{};
  ChassisMotion *chassis_motion_{};
  GimbalMotion *gimbal_motion_{};
};

} /* namespace */
#endif // ENGINEER_MIDDLEWARE_STEP_H_
