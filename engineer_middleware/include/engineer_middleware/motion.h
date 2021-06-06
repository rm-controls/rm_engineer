//
// Created by qiayuan on 5/29/21.
//

#ifndef ENGINEER_MIDDLEWARE_MOTION_H_
#define ENGINEER_MIDDLEWARE_MOTION_H_

#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rm_msgs/GimbalCmd.h>
#include <engineer_middleware/chassis_interface.h>

namespace engineer_middleware {
template<class Interface>
class MotionBase {
 public:
  MotionBase(const XmlRpc::XmlRpcValue &motion, Interface &interface) : interface_(interface) {
    tolerance_linear_ = xmlRpcGetDouble(motion, "tolerance_linear", 0.01);
    tolerance_angular_ = xmlRpcGetDouble(motion, "tolerance_angular", 0.02);
    time_out_ = xmlRpcGetDouble(motion, "time_out", 3.);
  };
  ~MotionBase() = default;
  virtual bool move() = 0;
  virtual bool isFinish() = 0;
  bool checkTimeout(ros::Duration period) {
    if (period.toSec() > time_out_) {
      ROS_ERROR("Step timeout,it should be finish in %f seconds", time_out_);
      return false;
    }
    return true;
  }
 protected:
  Interface &interface_;
  double tolerance_linear_{}, tolerance_angular_{}, time_out_{};
};

class MoveitMotionBase : public MotionBase<moveit::planning_interface::MoveGroupInterface> {
 public:
  MoveitMotionBase(const XmlRpc::XmlRpcValue &motion, moveit::planning_interface::MoveGroupInterface &interface)
      : MotionBase<moveit::planning_interface::MoveGroupInterface>(motion, interface) {
    speed_ = xmlRpcGetDouble(motion, "speed", 0.1);
    accel_ = xmlRpcGetDouble(motion, "accel", 0.1);
  }
  bool move() override {
    interface_.setMaxVelocityScalingFactor(speed_);
    interface_.setMaxAccelerationScalingFactor(accel_);
    return true;
  }
 protected:
  double speed_, accel_;
};

class EndEffectorMotion : public MoveitMotionBase {
 public:
  EndEffectorMotion(const XmlRpc::XmlRpcValue &motion,
                    moveit::planning_interface::MoveGroupInterface &interface, tf2_ros::Buffer &tf) :
      MoveitMotionBase(motion, interface), tf_(tf), has_pos_(false), has_ori_(false), is_cartesian_(false) {
    target_.pose.orientation.w = 1.;
    if (motion.hasMember("frame"))
      target_.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position")) {
      ROS_ASSERT(motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      target_.pose.position.x = xmlRpcGetDouble(motion["position"], 0, 0.0);
      target_.pose.position.y = xmlRpcGetDouble(motion["position"], 1, 0.0);
      target_.pose.position.z = xmlRpcGetDouble(motion["position"], 2, 0.0);
      has_pos_ = true;
    }
    if (motion.hasMember("rpy")) {
      ROS_ASSERT(motion["rpy"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(motion["rpy"][0], motion["rpy"][1], motion["rpy"][2]);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
      target_.pose.orientation = quat_msg;
      has_ori_ = true;
    }
    ROS_ASSERT(has_pos_ || has_ori_);
    if (motion.hasMember("cartesian")) is_cartesian_ = motion["cartesian"];
  }
  bool move() override {
    MoveitMotionBase::move();
    if (!target_.header.frame_id.empty() && target_.header.frame_id != interface_.getPlanningFrame()) {
      try {
        tf2::doTransform(target_.pose, target_.pose,
                         tf_.lookupTransform(interface_.getPlanningFrame(), target_.header.frame_id, ros::Time(0)));
        target_.header.frame_id = interface_.getPlanningFrame();
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
      }
    }
    if (is_cartesian_) {
      moveit_msgs::RobotTrajectory trajectory;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(target_.pose);
      if (interface_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory) < 99.9)
        return false;
      return interface_.asyncExecute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    } else {
      if (has_pos_ && has_ori_) interface_.setPoseTarget(target_);
      else if (has_pos_ && !has_ori_)
        interface_.setPositionTarget(target_.pose.position.x, target_.pose.position.y, target_.pose.position.z);
      else if (!has_pos_ && has_ori_)
        interface_.setOrientationTarget(target_.pose.orientation.x, target_.pose.orientation.y,
                                        target_.pose.orientation.z, target_.pose.orientation.w);
      return interface_.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
  }
  bool isFinish() override {
    geometry_msgs::Pose pose = interface_.getCurrentPose().pose;
    // TODO: Add orientation error check
    return (std::abs(std::pow(pose.position.x - target_.pose.position.x, 2)
                         + std::pow(pose.position.y - target_.pose.position.y, 2)
                         + std::pow(pose.position.z - target_.pose.position.z, 2)) < tolerance_linear_);
  }
 private:
  tf2_ros::Buffer &tf_;
  bool has_pos_, has_ori_, is_cartesian_;
  geometry_msgs::PoseStamped target_;
};

class JointMotion : public MoveitMotionBase {
 public:
  JointMotion(const XmlRpc::XmlRpcValue &motion, moveit::planning_interface::MoveGroupInterface &interface) :
      MoveitMotionBase(motion, interface) {
    if (motion.hasMember("joints")) {
      ROS_ASSERT(motion["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < motion["joints"].size(); ++i)
        target_.push_back(xmlRpcGetDouble(motion["joints"], i, 0.0));
    }
  }
  bool move() override {
    if (target_.empty())
      return false;
    MoveitMotionBase::move();
    interface_.setJointValueTarget(target_);
    return (interface_.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  bool isFinish() override {
    std::vector<double> current = interface_.getCurrentJointValues();
    double error = 0.;
    for (int i = 0; i < (int) current.size(); ++i)
      error += std::abs(target_[i] - current[i]);
    return error < tolerance_angular_;
  }
 private:
  std::vector<double> target_;
};

template<class MsgType>
class PublishMotion : public MotionBase<ros::Publisher> {
 public:
  PublishMotion(const XmlRpc::XmlRpcValue &motion, ros::Publisher &interface) :
      MotionBase<ros::Publisher>(motion, interface) {}
  bool move() override {
    interface_.publish(msg_);
    return true;
  }
  bool isFinish() override { return true; } // TODO: Add feedback
 protected:
  MsgType msg_;
};

class JointPositionMotion : public PublishMotion<std_msgs::Float64> {
 public:
  JointPositionMotion(const XmlRpc::XmlRpcValue &motion, ros::Publisher &interface) :
      PublishMotion<std_msgs::Float64>(motion, interface) {
    ROS_ASSERT(motion.hasMember("target"));
    msg_.data = xmlRpcGetDouble(motion, "target", 0.0);
  }
};

class GimbalMotion : public PublishMotion<rm_msgs::GimbalCmd> {
 public:
  GimbalMotion(const XmlRpc::XmlRpcValue &motion, ros::Publisher &interface) :
      PublishMotion<rm_msgs::GimbalCmd>(motion, interface) {
    if (motion.hasMember("frame"))
      msg_.aim_point.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position")) {
      ROS_ASSERT(motion["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      msg_.aim_point.point.x = xmlRpcGetDouble(motion["position"], 0, 0.0);
      msg_.aim_point.point.y = xmlRpcGetDouble(motion["position"], 1, 0.0);
      msg_.aim_point.point.z = xmlRpcGetDouble(motion["position"], 2, 0.0);
    }
    msg_.mode = msg_.DIRECT;
  }
};

class ChassisMotion : public MotionBase<ChassisInterface> {
 public:
  ChassisMotion(const XmlRpc::XmlRpcValue &motion, ChassisInterface &interface)
      : MotionBase<ChassisInterface>(motion, interface) {
    if (motion.hasMember("frame"))
      target_.header.frame_id = std::string(motion["frame"]);
    if (motion.hasMember("position")) {
      target_.pose.position.x = xmlRpcGetDouble(motion["position"], 0, 0.0);
      target_.pose.position.y = xmlRpcGetDouble(motion["position"], 1, 0.0);
    }
    if (motion.hasMember("yaw")) {
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(0, 0, motion["yaw"]);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
      target_.pose.orientation = quat_msg;
    }
  }
  bool move() override { return interface_.setGoal(target_); }
  bool isFinish() override {
    return interface_.getPosError() < tolerance_linear_ && interface_.getYawError() < tolerance_angular_;
  }
  geometry_msgs::PoseStamped target_;
};

}

#endif //ENGINEER_MIDDLEWARE_MOTION_H_
