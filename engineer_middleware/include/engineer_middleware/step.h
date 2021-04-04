#pragma once

#include "arm_motion.h"

// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include <rm_common/ros_utilities.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace engineer_middleware {

class LegMotionBase;
class BaseMotionBase;

class Step {
 public:
  Step(const XmlRpc::XmlRpcValue &step, moveit::planning_interface::MoveGroupInterface &arm_group,
       moveit::planning_interface::MoveGroupInterface &hand_group);
  ~Step() = default;

  bool compute(const moveit::core::RobotState &current_state);

  bool hasArmMotion() const { return (bool) (arm_motion_); }
  const BaseMotionBase &getBaseMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getBaseMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };
  bool hasBaseMotion() const { return (bool) (base_motion_); };
  const BaseMotionBase &getArmMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getArmMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };

  const std::string &getId() const { return id_; }
  void setId(const std::string &id) { id_ = id; }

  friend class StepCompleter;

 protected:
  std::unique_ptr<ArmMotionBase> arm_motion_;
  std::unique_ptr<BaseMotionBase> base_motion_;

 private:
  std::string id_;
};

} /* namespace */
