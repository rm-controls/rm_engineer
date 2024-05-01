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
  void init()
  {
    state_ = SET_GOAL;
    initComputeValue();
  }
  void stateMachine()
  {
    switch (state_)
    {
      case SET_GOAL:
      {
        setGoal();
        state_ = ADJUST_X;
      }
      break;
      case ADJUST_X:
      {
        computeChassisVel();
        chassis_vel_cmd_.linear.y = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (x_.isFinish())
          state_ = re_adjusted_ ? FINISH : ADJUST_Y;
      }
      break;
      case ADJUST_Y:
      {
        computeChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (y_.isFinish())
          state_ = ADJUST_YAW;
      }
      break;
      case ADJUST_YAW:
      {
        computeChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.linear.y = 0.;
        if (yaw_.isFinish())
        {
          state_ = SET_GOAL;
          re_adjusted_ = true;
        }
      }
      break;
      case FINISH:
      {
        is_finish_ = true;
        re_adjusted_ = false;
        initComputeValue();
      }
      break;
    }
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
  void computeChassisVel()
  {
    geometry_msgs::TransformStamped current;
    current = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    x_.error = chassis_target_.pose.position.x - current.transform.translation.x;
    y_.error = chassis_target_.pose.position.y - current.transform.translation.y;
    double roll, pitch, yaw_current, yaw_goal;
    quatToRPY(current.transform.rotation, roll, pitch, yaw_current);
    quatToRPY(chassis_target_.pose.orientation, roll, pitch, yaw_goal);
    yaw_.error = angles::shortest_angular_distance(yaw_current, yaw_goal);

    ros::Duration dt = ros::Time::now() - last_time_;
    chassis_vel_cmd_.linear.x = x_.computerVel(dt);
    chassis_vel_cmd_.linear.y = y_.computerVel(dt);
    chassis_vel_cmd_.angular.z = yaw_.computerVel(dt);

    last_time_ = ros::Time::now();
  }
  void setGoal()
  {
    geometry_msgs::TransformStamped base2exchange;
    double roll, pitch, yaw;
    base2exchange = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
    quatToRPY(base2exchange.transform.rotation, roll, pitch, yaw);

    double goal_x = base2exchange.transform.translation.x - x_.offset_refer_exchanger;
    double goal_y = base2exchange.transform.translation.y - y_.offset_refer_exchanger;
    double goal_yaw = yaw * yaw_.offset_refer_exchanger;
    chassis_original_target_.pose.position.x = goal_x;
    chassis_original_target_.pose.position.y = goal_y;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, goal_yaw);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    chassis_original_target_.pose.orientation = quat_msg;
    chassis_target_ = chassis_original_target_;
    tf2::doTransform(chassis_target_, chassis_target_, tf_buffer_.lookupTransform("map", "base_link", ros::Time(0)));
  }
  bool re_adjusted_{ false }, is_finish_{ false };
  AdjustStates state_, last_state_;
  tf2_ros::Buffer& tf_buffer_;
  ros::NodeHandle nh_{};
  ros::Time last_time_;
  SingleDirectionMove x_, y_, yaw_;
  geometry_msgs::Twist chassis_vel_cmd_{};
  geometry_msgs::PoseStamped chassis_target_{}, chassis_original_target_{};
};

class MoveitExchange
{
};

}  // namespace moveit_exchange
