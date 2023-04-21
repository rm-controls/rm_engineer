/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 5/29/21.
//

#pragma once

#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/GpioData.h>
#include <rm_msgs/MultiDofCmd.h>
#include <std_msgs/Int32.h>
#include <engineer_middleware/chassis_interface.h>

namespace engineer_middleware
{
template <class Interface>
class MotionBase
{
public:
  MotionBase(XmlRpc::XmlRpcValue& motion, Interface& interface) : interface_(interface)
  {
    time_out_ = xmlRpcGetDouble(motion["common"], "timeout", 1e10);
  };
  ~MotionBase() = default;
  virtual bool move() = 0;
  virtual bool isFinish() = 0;
  bool checkTimeout(ros::Duration period)
  {
    if (period.toSec() > time_out_)
    {
      ROS_ERROR("Step timeout,it should be finish in %f seconds", time_out_);
      return false;
    }
    return true;
  }
  virtual void stop() = 0;

protected:
  Interface& interface_;
  double time_out_{};
};

class MoveitMotionBase : public MotionBase<moveit::planning_interface::MoveGroupInterface>
{
public:
  MoveitMotionBase(XmlRpc::XmlRpcValue& motion, moveit::planning_interface::MoveGroupInterface& interface)
    : MotionBase<moveit::planning_interface::MoveGroupInterface>(motion, interface)
  {
    speed_ = xmlRpcGetDouble(motion["common"], "speed", 0.1);
    accel_ = xmlRpcGetDouble(motion["common"], "accel", 0.1);
  }
  bool move() override
  {
    interface_.setMaxVelocityScalingFactor(speed_);
    interface_.setMaxAccelerationScalingFactor(accel_);
    countdown_ = 5;
    return true;
  }
  bool isFinish() override
  {
    if (isReachGoal())
      countdown_--;
    else
      countdown_ = 5;
    return countdown_ < 0;
  }
  void stop() override
  {
    interface_.setMaxVelocityScalingFactor(0.);
    interface_.setMaxAccelerationScalingFactor(0.);
    interface_.stop();
  }
  std_msgs::Int32 judgePlanningResult()
  {
    return msg_;
  }

protected:
  virtual bool isReachGoal() = 0;
  double speed_, accel_;
  int countdown_{};
  std_msgs::Int32 msg_;
};

class EndEffectorMotion : public MoveitMotionBase
{
public:
  EndEffectorMotion(XmlRpc::XmlRpcValue& motion, moveit::planning_interface::MoveGroupInterface& interface,
                    tf2_ros::Buffer& tf)
    : MoveitMotionBase(motion, interface), tf_(tf), has_pos_(false), has_ori_(false), is_cartesian_(false)
  {
    target_.pose.orientation.w = 1.;
    tolerance_position_ = xmlRpcGetDouble(motion, "tolerance_position", 0.01);
    tolerance_orientation_ = xmlRpcGetDouble(motion, "tolerance_orientation", 0.1);
    if (motion.hasMember("frame"))
      target_.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position"))
    {
      ROS_ASSERT(motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      target_.pose.position.x = xmlRpcGetDouble(motion["position"], 0);
      target_.pose.position.y = xmlRpcGetDouble(motion["position"], 1);
      target_.pose.position.z = xmlRpcGetDouble(motion["position"], 2);
      has_pos_ = true;
    }
    if (motion.hasMember("rpy"))
    {
      ROS_ASSERT(motion["rpy"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(motion["rpy"][0], motion["rpy"][1], motion["rpy"][2]);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
      target_.pose.orientation = quat_msg;
      has_ori_ = true;
    }
    ROS_ASSERT(has_pos_ || has_ori_);
    if (motion.hasMember("cartesian"))
      is_cartesian_ = motion["cartesian"];
  }
  bool move() override
  {
    MoveitMotionBase::move();
    geometry_msgs::PoseStamped final_target;
    if (!target_.header.frame_id.empty() && target_.header.frame_id != interface_.getPlanningFrame())
    {
      try
      {
        tf2::doTransform(target_.pose, final_target.pose,
                         tf_.lookupTransform(interface_.getPlanningFrame(), target_.header.frame_id, ros::Time(0)));
        final_target.header.frame_id = interface_.getPlanningFrame();
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return false;
      }
    }
    if (is_cartesian_)
    {
      moveit_msgs::RobotTrajectory trajectory;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(target_.pose);
      if (interface_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory) < 99.9)
        return false;
      return interface_.asyncExecute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
    else
    {
      if (has_pos_ && has_ori_)
        interface_.setPoseTarget(final_target);
      else if (has_pos_ && !has_ori_)
        interface_.setPositionTarget(final_target.pose.position.x, final_target.pose.position.y,
                                     final_target.pose.position.z);
      else if (!has_pos_ && has_ori_)
        interface_.setOrientationTarget(final_target.pose.orientation.x, final_target.pose.orientation.y,
                                        final_target.pose.orientation.z, final_target.pose.orientation.w);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      msg_.data = interface_.plan(plan).val;
      return interface_.asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
  }

private:
  bool isReachGoal() override
  {
    geometry_msgs::Pose pose = interface_.getCurrentPose().pose;
    double roll_current, pitch_current, yaw_current, roll_goal, pitch_goal, yaw_goal;
    quatToRPY(pose.orientation, roll_current, pitch_current, yaw_current);
    quatToRPY(target_.pose.orientation, roll_goal, pitch_goal, yaw_goal);
    // TODO: Add orientation error check
    return (std::pow(pose.position.x - target_.pose.position.x, 2) +
                    std::pow(pose.position.y - target_.pose.position.y, 2) +
                    std::pow(pose.position.z - target_.pose.position.z, 2) <
                tolerance_position_ &&
            std::abs(angles::shortest_angular_distance(yaw_current, yaw_goal)) +
                    std::abs(angles::shortest_angular_distance(pitch_current, pitch_goal)) +
                    std::abs(angles::shortest_angular_distance(yaw_current, yaw_goal)) <
                tolerance_orientation_);
  }
  tf2_ros::Buffer& tf_;
  bool has_pos_, has_ori_, is_cartesian_;
  geometry_msgs::PoseStamped target_;
  double tolerance_position_, tolerance_orientation_;
};

class JointMotion : public MoveitMotionBase
{
public:
  JointMotion(XmlRpc::XmlRpcValue& motion, moveit::planning_interface::MoveGroupInterface& interface)
    : MoveitMotionBase(motion, interface)
  {
    if (motion.hasMember("joints"))
    {
      ROS_ASSERT(motion["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < motion["joints"].size(); ++i)
      {
        ROS_ASSERT(motion["joints"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble || motion["joints"][i] == "KEEP");
        if (motion["joints"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          target_.push_back(motion["joints"][i]);
        else if (motion["joints"][i] == "KEEP")
          target_.push_back(NAN);
      }
    }
    if (motion.hasMember("tolerance"))
    {
      ROS_ASSERT(motion["tolerance"]["tolerance_joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < motion["tolerance"]["tolerance_joints"].size(); ++i)
        tolerance_joints_.push_back(xmlRpcGetDouble(motion["tolerance"]["tolerance_joints"], i));
    }
  }
  bool move() override
  {
    if (target_.empty())
      return false;
    MoveitMotionBase::move();
    for (int i = 0; i < (int)target_.size(); i++)
    {
      if (!std::isnormal(target_[i]))
      {
        target_[i] = interface_.getCurrentJointValues()[i];
      }
    }
    interface_.setJointValueTarget(target_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    interface_.plan(plan);
    msg_.data = interface_.plan(plan).val;
    return (interface_.asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

private:
  bool isReachGoal() override
  {
    std::vector<double> current = interface_.getCurrentJointValues();
    double error = 0.;
    bool flag = 1;
    for (int i = 0; i < (int)target_.size(); ++i)
    {
      error = std::abs(target_[i] - current[i]);
      flag &= (error < tolerance_joints_[i]);
    }
    return flag;
  }
  std::vector<double> target_, final_target_, tolerance_joints_;
};

class VisMotion : public MoveitMotionBase
{
public:
  VisMotion(XmlRpc::XmlRpcValue& motion, moveit::planning_interface::MoveGroupInterface& interface, tf2_ros::Buffer& tf)
    : MoveitMotionBase(motion, interface), tf_(tf), has_pos_(false), has_ori_(false)
  {
    target_.pose.orientation.w = 1.;
    tolerance_position_ = xmlRpcGetDouble(motion, "tolerance_position", 0.01);
    tolerance_orientation_ = xmlRpcGetDouble(motion, "tolerance_orientation", 0.1);
    has_pos_ = true;
    has_ori_ = true;
  }
  bool moveing(geometry_msgs::TwistStamped test)
  {
    geometry_msgs::PoseStamped temp_;
    target_.header.frame_id = test.header.frame_id;
    target_.pose.position.x = test.twist.linear.x;
    target_.pose.position.y = test.twist.linear.y;
    target_.pose.position.z = test.twist.linear.z;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(test.twist.angular.x, test.twist.angular.y, test.twist.angular.z);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    target_.pose.orientation = quat_msg;
    if (!target_.header.frame_id.empty() && target_.header.frame_id != interface_.getPlanningFrame())
    {
      try
      {
        tf2::doTransform(target_.pose, final_target_.pose,
                         tf_.lookupTransform(interface_.getPlanningFrame(), target_.header.frame_id, ros::Time(0)));
        final_target_.header.frame_id = interface_.getPlanningFrame();
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return false;
      }
    }
    if (has_pos_ && has_ori_)
      interface_.setPoseTarget(final_target_);
    else if (has_pos_ && !has_ori_)
      interface_.setPositionTarget(final_target_.pose.position.x, final_target_.pose.position.y,
                                   final_target_.pose.position.z);
    else if (!has_pos_ && has_ori_)
      interface_.setOrientationTarget(final_target_.pose.orientation.x, final_target_.pose.orientation.y,
                                      final_target_.pose.orientation.z, final_target_.pose.orientation.w);
    MoveitMotionBase::move();
    return interface_.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }

private:
  bool isReachGoal() override
  {
    geometry_msgs::Pose pose = interface_.getCurrentPose().pose;
    double roll_current, pitch_current, yaw_current, roll_goal, pitch_goal, yaw_goal;
    quatToRPY(pose.orientation, roll_current, pitch_current, yaw_current);
    quatToRPY(final_target_.pose.orientation, roll_goal, pitch_goal, yaw_goal);
    // TODO: Add orientation error check
    return (std::pow(pose.position.x - final_target_.pose.position.x, 2) +
                    std::pow(pose.position.y - final_target_.pose.position.y, 2) +
                    std::pow(pose.position.z - final_target_.pose.position.z, 2) <
                tolerance_position_ &&
            std::abs(angles::shortest_angular_distance(yaw_current, yaw_goal)) +
                    std::abs(angles::shortest_angular_distance(pitch_current, pitch_goal)) +
                    std::abs(angles::shortest_angular_distance(yaw_current, yaw_goal)) <
                tolerance_orientation_);
  }
  tf2_ros::Buffer& tf_;
  bool has_pos_, has_ori_;
  geometry_msgs::PoseStamped target_, final_target_;
  double tolerance_position_, tolerance_orientation_;
};

template <class MsgType>
class PublishMotion : public MotionBase<ros::Publisher>
{
public:
  PublishMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface) : MotionBase<ros::Publisher>(motion, interface)
  {
  }
  bool move() override
  {
    interface_.publish(msg_);
    return true;
  }
  bool isFinish() override
  {
    return true;
  }  // TODO: Add feedback
  void stop() override
  {
  }

protected:
  MsgType msg_;
};

class HandMotion : public PublishMotion<std_msgs::Float64>
{
public:
  HandMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface)
    : PublishMotion<std_msgs::Float64>(motion, interface)
  {
    ROS_ASSERT(motion.hasMember("position"));
    ROS_ASSERT(motion.hasMember("delay"));
    position_ = xmlRpcGetDouble(motion, "position", 0.0);
    delay_ = xmlRpcGetDouble(motion, "delay", 0.0);
  }
  bool move() override
  {
    start_time_ = ros::Time::now();
    msg_.data = position_;
    return PublishMotion::move();
  }
  bool isFinish() override
  {
    return ((ros::Time::now() - start_time_).toSec() > delay_);
  }

private:
  double position_, delay_;
  ros::Time start_time_;
};

class GpioMotion : public PublishMotion<rm_msgs::GpioData>
{
public:
  GpioMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface)
    : PublishMotion<rm_msgs::GpioData>(motion, interface)
  {
    state_ = motion["state"];
    msg_.gpio_state.push_back(0);
    msg_.gpio_name.push_back("gripper");
  }
  bool move() override
  {
    msg_.gpio_state[0] = state_;
    return PublishMotion::move();
  }

private:
  bool state_;
};

class JointPositionMotion : public PublishMotion<std_msgs::Float64>
{
public:
  JointPositionMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface)
    : PublishMotion<std_msgs::Float64>(motion, interface)
  {
    ROS_ASSERT(motion.hasMember("target"));
    msg_.data = xmlRpcGetDouble(motion, "target", 0.0);
  }
};

class GimbalMotion : public PublishMotion<rm_msgs::GimbalCmd>
{
public:
  GimbalMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface)
    : PublishMotion<rm_msgs::GimbalCmd>(motion, interface)
  {
    if (motion.hasMember("frame"))
      msg_.target_pos.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position"))
    {
      ROS_ASSERT(motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      msg_.target_pos.point.x = xmlRpcGetDouble(motion["position"], 0);
      msg_.target_pos.point.y = xmlRpcGetDouble(motion["position"], 1);
      msg_.target_pos.point.z = xmlRpcGetDouble(motion["position"], 2);
    }
    msg_.mode = msg_.DIRECT;
  }
};

class ChassisMotion : public MotionBase<ChassisInterface>
{
public:
  ChassisMotion(XmlRpc::XmlRpcValue& motion, ChassisInterface& interface)
    : MotionBase<ChassisInterface>(motion, interface)
  {
    chassis_tolerance_position_ = xmlRpcGetDouble(motion, "chassis_tolerance_position_", 0.01);
    chassis_tolerance_angular_ = xmlRpcGetDouble(motion, "chassis_tolerance_angular_", 0.01);
    if (motion.hasMember("frame"))
      target_.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position"))
    {
      target_.pose.position.x = xmlRpcGetDouble(motion["position"], 0);
      target_.pose.position.y = xmlRpcGetDouble(motion["position"], 1);
    }
    if (motion.hasMember("yaw"))
    {
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(0, 0, motion["yaw"]);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
      target_.pose.orientation = quat_msg;
    }
  }
  bool move() override
  {
    interface_.setGoal(target_);
    return true;
  }
  bool isFinish() override
  {
    return interface_.getErrorPos() < chassis_tolerance_position_ &&
           interface_.getErrorYaw() < chassis_tolerance_angular_;
  }
  void stop() override
  {
    interface_.stop();
  }

private:
  geometry_msgs::PoseStamped target_;
  double chassis_tolerance_position_, chassis_tolerance_angular_;
};

class ReversalMotion : public PublishMotion<rm_msgs::MultiDofCmd>
{
public:
  ReversalMotion(XmlRpc::XmlRpcValue& motion, ros::Publisher& interface)
    : PublishMotion<rm_msgs::MultiDofCmd>(motion, interface)
  {
    delay_ = xmlRpcGetDouble(motion, "delay", 0.0);
    if (std::string(motion["mode"]) == "POSITION")
      msg_.mode = msg_.POSITION;
    else
      msg_.mode = msg_.VELOCITY;
    if (motion.hasMember("values"))
    {
      ROS_ASSERT(motion["values"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      msg_.linear.x = xmlRpcGetDouble(motion["values"], 0);
      msg_.linear.y = xmlRpcGetDouble(motion["values"], 1);
      msg_.linear.z = xmlRpcGetDouble(motion["values"], 2);
      msg_.angular.x = xmlRpcGetDouble(motion["values"], 3);
      msg_.angular.y = xmlRpcGetDouble(motion["values"], 4);
      msg_.angular.z = xmlRpcGetDouble(motion["values"], 5);
    }
  }
  void setZero()
  {
    zero_msg_.mode = msg_.mode;
    zero_msg_.linear.x = 0.;
    zero_msg_.linear.y = 0.;
    zero_msg_.linear.z = 0.;
    zero_msg_.angular.x = 0.;
    zero_msg_.angular.y = 0.;
    zero_msg_.angular.z = 0.;
  }
  bool move() override
  {
    start_time_ = ros::Time::now();
    interface_.publish(msg_);
    if (msg_.mode == msg_.POSITION)
    {
      t.sleep();
      ReversalMotion::setZero();
      interface_.publish(zero_msg_);
    }
    return true;
  }
  bool isFinish() override
  {
    return ((ros::Time::now() - start_time_).toSec() > delay_);
  }

private:
  double delay_;
  ros::Duration t = ros::Duration(0.2);
  ros::Time start_time_;
  rm_msgs::MultiDofCmd zero_msg_;
};
}  // namespace engineer_middleware
