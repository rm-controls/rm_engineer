//
// Created by qiayuan on 4/5/21.
//

#include "engineer_middleware/arm_motion.h"

namespace engineer_middleware {

ArmMotionBase::ArmMotionBase(const XmlRpc::XmlRpcValue &arm_motion,
                             moveit::planning_interface::MoveGroupInterface &arm_group,
                             moveit::planning_interface::MoveGroupInterface &hand_group)
    : arm_group_(arm_group), hand_group_(hand_group) {
  if (arm_motion.hasMember("hand_open")) {
    if (arm_motion["hand_open"] == "before")
      hand_motion_ = OPEN_BEFORE_ARM_MOTION;
    if (arm_motion["hand_open"] == "after")
      hand_motion_ = OPEN_AFTER_ARM_MOTION;
  } else if (arm_motion.hasMember("hand_close")) {
    if (arm_motion["hand_close"] == "before")
      hand_motion_ = CLOSE_BEFORE_ARM_MOTION;
    if (arm_motion["hand_close"] == "after")
      hand_motion_ = CLOSE_AFTER_ARM_MOTION;
  } else {
    hand_motion_ = FREEZE;
  }
}
bool ArmMotionBase::compute(const moveit::core::RobotState &current_state) {
  hand_group_.setStartState(current_state);
  if (hand_motion_ > 0 && hand_motion_ < 3)
    hand_group_.setJointValueTarget("right_finger_joint", 0.0);
  else if (hand_motion_ > 2)
    hand_group_.setJointValueTarget("right_finger_joint", 0.02);
  if (hand_motion_ == FREEZE)
    return true;
  else
    return (hand_group_.plan(hand_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

EndEffectorTarget::EndEffectorTarget(const XmlRpc::XmlRpcValue &arm_motion,
                                     moveit::planning_interface::MoveGroupInterface &arm_group,
                                     moveit::planning_interface::MoveGroupInterface &hand_group)
    : ArmMotionBase(arm_motion, arm_group, hand_group) {
  target_.pose.orientation.w = 1.;
  if (arm_motion.hasMember("frame"))
    target_.header.frame_id = std::string(arm_motion["frame"]);
  if (arm_motion.hasMember("position")) {
    ROS_ASSERT(arm_motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    target_.pose.position.x = arm_motion["position"][0];
    target_.pose.position.y = arm_motion["position"][1];
    target_.pose.position.z = arm_motion["position"][2];
    has_pos_ = true;
  }
  if (arm_motion.hasMember("rpy")) {
    ROS_ASSERT(arm_motion["rpy"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(arm_motion["rpy"][0], arm_motion["rpy"][1], arm_motion["rpy"][2]);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    target_.pose.orientation = quat_msg;
    has_ori_ = true;
  }
}

bool EndEffectorTarget::compute(const moveit::core::RobotState &current_state) {
  arm_group_.setStartState(current_state);
  if (!has_pos_ && !has_ori_)
    return true;
  if (has_pos_ && has_pos_)
    arm_group_.setPoseTarget(target_);
  if (has_pos_ && !has_ori_)
    arm_group_.setPositionTarget(target_.pose.position.x, target_.pose.position.y, target_.pose.position.z);
  if (!has_pos_ && has_ori_)
    arm_group_.setOrientationTarget(target_.pose.orientation.x,
                                    target_.pose.orientation.y,
                                    target_.pose.orientation.z,
                                    target_.pose.orientation.w);
  return (arm_group_.plan(arm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) &&
      ArmMotionBase::compute(current_state);
}

}