#pragma once

#include "engineer_middleware/motion.h"
// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

namespace engineer_middleware {

class Step {
 public:
  Step(const XmlRpc::XmlRpcValue &step, tf2_ros::Buffer &tf,
       moveit::planning_interface::MoveGroupInterface &arm_group,
       moveit::planning_interface::MoveGroupInterface &hand_group,
       ros::Publisher &card_pub, ros::Publisher &gimbal_pub) {
    if (step.hasMember("arm")) {
      if (step.hasMember("joints"))
        arm_motion_ = new JointMotion(step["arm"], arm_group);
      else
        arm_motion_ = new EndEffectorMotion(step["arm"], arm_group, tf);
    }
    if (step.hasMember("hand"))
      hand_motion_ = new JointMotion(step["hand"], hand_group);
    if (step.hasMember("card"))
      card_motion_ = new JointPositionMotion(step["card"], card_pub);
    if (step.hasMember("gimbal"))
      gimbal_motion_ = new GimbalMotion(step["gimbal"], gimbal_pub);
  }
  bool move() {
    bool success = true;
    if (arm_motion_) success &= arm_motion_->move();
    if (arm_motion_) success &= hand_motion_->move();
    if (arm_motion_) success &= card_motion_->move();
    if (arm_motion_) success &= arm_motion_->move();
    if (arm_motion_) success &= gimbal_motion_->move();
    return success;
  }
  bool isFinish() {
    // TODO Add timeout
    bool success = true;
    if (arm_motion_) success &= arm_motion_->isFinish();
    if (arm_motion_) success &= hand_motion_->isFinish();
    if (arm_motion_) success &= card_motion_->isFinish();
    if (arm_motion_) success &= arm_motion_->isFinish();
    if (arm_motion_) success &= gimbal_motion_->isFinish();
    return success;
  }
 private:
  MoveitMotionBase *arm_motion_{}, *hand_motion_{};
  JointPositionMotion *card_motion_{};
  GimbalMotion *gimbal_motion_{};
};

} /* namespace */
