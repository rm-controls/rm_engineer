//
// Created by cch on 24-4-27.
//

#pragma once
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <engineer_middleware/motion.h>

namespace moveit_exchange
{
enum States
{
  FIND,
  CHASSIS_APPROACH,
  CHASSIS_POMPENSATE,
  SPHERE,
  LINE,
  REACH
};

class SingleDirectionMove
{
public:
  std::string name;
  double tolerance{}, start_vel{}, offset_refer_exchanger{}, max_vel{}, error{}, pid_value{};
  control_toolbox::Pid pid;
  void init(XmlRpc::XmlRpcValue& config, std::string config_name, ros::NodeHandle& nh)
  {
    name = config_name;
    error = 1e10;
    max_vel = config.hasMember("max_vel") ? (double)config["max_vel"] : 1e10;
    start_vel = config.hasMember("start_vel") ? (double)config["start_vel"] : 0.;
    tolerance = config.hasMember("tolerance") ? (double)config["tolerance"] : 1e10;
    offset_refer_exchanger = config.hasMember("offset_refer_exchanger") ? (double)config["offset_refer_exchanger"] : 0.;
    ros::NodeHandle pid_config = ros::NodeHandle(nh, name);
    pid.init(ros::NodeHandle(pid_config, "pid"));
  }
  bool isFinish() const
  {
    return abs(error) <= tolerance;
  }
  double computerVel(ros::Duration dt)
  {
    double vel = start_vel + abs(pid.computeCommand(error, dt));
    int direction = (error > 0) ? 1 : -1;
    return abs(vel) >= max_vel ? direction * max_vel : direction * vel;
  }
  void getPidValue(ros::Duration dt)
  {
    double vel = start_vel + abs(pid.computeCommand(error, dt));
    int direction = (error > 0) ? 1 : -1;
    pid_value = abs(vel) >= max_vel ? direction * max_vel : direction * vel;
  }
};

class ChassisMotion
{
public:
  enum AdjustStates
  {
    SET_GOAL,
    ADJUST_X,
    ADJUST_Y,
    ADJUST_YAW,
    FINISH
  };
  ChassisMotion(XmlRpc::XmlRpcValue& config, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : tf_buffer_(tf_buffer), nh_(nh)
  {
    state_ = SET_GOAL;
    last_state_ = state_;
    x_.init(config["x"], "x", nh);
    y_.init(config["y"], "y", nh);
    yaw_.init(config["yaw"], "yaw", nh);
  }

private:
  void initComputeValue()
  {
    chassis_vel_cmd_.linear.x = 0;
    chassis_vel_cmd_.linear.y = 0;
    chassis_vel_cmd_.linear.z = 0;
    chassis_vel_cmd_.angular.x = 0;
    chassis_vel_cmd_.angular.y = 0;
    chassis_vel_cmd_.angular.z = 0;
  }

  AdjustStates state_, last_state_;
  tf2_ros::Buffer& tf_buffer_;
  ros::NodeHandle nh_{};
  SingleDirectionMove x_, y_, yaw_;
  geometry_msgs::Twist chassis_vel_cmd_{};
};

class FindExchanger
{
};

class MoveitExchange
{
};

}  // namespace moveit_exchange
