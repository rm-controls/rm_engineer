//
// Created by qiayuan on 4/5/21.
//

#include "engineer_middleware/arm_motion.h"
#include <rm_common/ros_utilities.h>

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
  if (hand_motion_ > 0 && hand_motion_ < 3)
    hand_group_.setJointValueTarget("right_finger_joint", 0.0);
  else if (hand_motion_ > 2)
    hand_group_.setJointValueTarget("right_finger_joint", 0.01);
  if (hand_motion_ == FREEZE)
    return true;
  else
    return (hand_group_.plan(hand_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool ArmMotionBase::move() {
  if (hand_motion_)
    return hand_group_.execute(hand_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  else
    return true;
}

EndEffectorTarget::EndEffectorTarget(const XmlRpc::XmlRpcValue &arm_motion,
                                     moveit::planning_interface::MoveGroupInterface &arm_group,
                                     moveit::planning_interface::MoveGroupInterface &hand_group)
    : ArmMotionBase(arm_motion, arm_group, hand_group) {
  has_pos_ = false;
  has_ori_ = false;
  is_cartesian_ = false;

  target_.pose.orientation.w = 1.;
  if (arm_motion.hasMember("frame"))
    target_.header.frame_id = std::string(arm_motion["frame"]);
  if (arm_motion.hasMember("position")) {
    ROS_ASSERT(arm_motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    target_.pose.position.x = xmlRpcGetDouble(arm_motion["position"], 0, 0.0);
    target_.pose.position.y = xmlRpcGetDouble(arm_motion["position"], 1, 0.0);
    target_.pose.position.z = xmlRpcGetDouble(arm_motion["position"], 2, 0.0);
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
  if (arm_motion.hasMember("cartesian"))
    is_cartesian_ = arm_motion["cartesian"];
}

bool EndEffectorTarget::compute(const moveit::core::RobotState &current_state) {
  if (is_cartesian_) {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_.pose);
    if (arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory_) > 99.9)
      return true;
    else
      return false;
  } else {
    if (!has_pos_ && !has_ori_)
      return ArmMotionBase::compute(current_state);
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

bool EndEffectorTarget::move() {
  if (has_pos_ || has_ori_) {
    if (hand_motion_ == CLOSE_BEFORE_ARM_MOTION || hand_motion_ == OPEN_BEFORE_ARM_MOTION) {
      if (ArmMotionBase::move())
        return (is_cartesian_ ? arm_group_.execute(trajectory_) : arm_group_.execute(arm_plan_))
            == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      else
        return false;
    } else {
      if ((is_cartesian_ ? arm_group_.execute(trajectory_) : arm_group_.execute(arm_plan_))
          == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return ArmMotionBase::move();
      else
        return false;
    }
  } else
    return ArmMotionBase::move();
}

JointsTarget::JointsTarget(const XmlRpc::XmlRpcValue &arm_motion,
                           moveit::planning_interface::MoveGroupInterface &arm_group,
                           moveit::planning_interface::MoveGroupInterface &hand_group)
    : ArmMotionBase(arm_motion, arm_group, hand_group) {
  if (arm_motion.hasMember("joints")) {
    ROS_ASSERT(arm_motion["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    target_.push_back(xmlRpcGetDouble(arm_motion["joints"], 0, 0.0));
    target_.push_back(xmlRpcGetDouble(arm_motion["joints"], 1, 0.0));
    target_.push_back(xmlRpcGetDouble(arm_motion["joints"], 2, 0.0));
    target_.push_back(xmlRpcGetDouble(arm_motion["joints"], 3, 0.0));
    target_.push_back(xmlRpcGetDouble(arm_motion["joints"], 4, 0.0));
    has_joints_ = true;
  }
}

bool JointsTarget::compute(const moveit::core::RobotState &current_state) {
  if (!has_joints_)
    return ArmMotionBase::compute(current_state);
  else {
    arm_group_.setJointValueTarget(target_);
    return (arm_group_.plan(arm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) &&
        ArmMotionBase::compute(current_state);
  }
}

bool JointsTarget::move() {
  if (has_joints_) {
    if (hand_motion_ == CLOSE_BEFORE_ARM_MOTION || hand_motion_ == OPEN_BEFORE_ARM_MOTION) {
      if (ArmMotionBase::move())
        return arm_group_.execute(arm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    } else {
      if (arm_group_.execute(arm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return ArmMotionBase::move();
    }
  }
  return ArmMotionBase::move();
}

}