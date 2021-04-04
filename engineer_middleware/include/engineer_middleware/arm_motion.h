#pragma once

#include "engineer_middleware/step.h"
#include "engineer_middleware/state.h"

// STD
#include <string>
#include <memory>
#include <geometry_msgs/Pose.h>

// ROS
#include <moveit/move_group_interface/move_group_interface.h>

namespace engineer_middleware {

enum HandMotion {
  CLOSE_AFTER_ARM_MOTION,
  CLOSE_BEFORE_ARM_MOTION,
  OPEN_AFTER_ARM_MOTION,
  OPEN_BEFORE_ARM_MOTION,
  FREEZE
};

class ArmMotionBase {
 public:
  ArmMotionBase(const XmlRpc::XmlRpcValue &arm_motion,
                moveit::planning_interface::MoveGroupInterface &arm_group,
                moveit::planning_interface::MoveGroupInterface &hand_group)
      : arm_group_(arm_group), hand_group_(hand_group) {}
  ~ArmMotionBase() = default;
  virtual bool compute(const moveit::core::RobotState &current_state) {}
 protected:
  moveit::planning_interface::MoveGroupInterface &arm_group_, &hand_group_;
};

class EndEffectorTarget : public ArmMotionBase {
 public:
  EndEffectorTarget(const XmlRpc::XmlRpcValue &arm_motion,
                    moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group) :
      ArmMotionBase(arm_motion, arm_group, hand_group) {

  }
 private:
  geometry_msgs::Pose target_;
};

class JointTarget : public ArmMotionBase {
 public:
  JointTarget(const XmlRpc::XmlRpcValue &arm_motion,
              moveit::planning_interface::MoveGroupInterface &arm_group,
              moveit::planning_interface::MoveGroupInterface &hand_group) :
      ArmMotionBase(arm_motion, arm_group, hand_group) {
  }
 private:
  std::vector<double> target_;
};

} /* namespace */
