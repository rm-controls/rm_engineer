//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>

namespace auto_exchange
{
enum FindProcess
{
  SWING,
  FOUND,
  LOCKED
};

enum AdjustProcess
{
  COMPUTER,
  MOVE_CHASSIS
};

enum ServoMoveProcess
{
  YZ,
  ROLL_YAW,
  XYZ,
  PITCH,
  PUSH,
  DONE
};

enum MotionMoveProcess
{
  SPHERE,
  LINE,
  POINT,
  ACHIEVE
};

enum ExchangeProcess
{
  FIND,
  PRE_ADJUST,
  MOVE,
  POST_ADJUST,
  FINISH
};

class JointInfo
{
public:
  JointInfo(XmlRpc::XmlRpcValue& joint)
  {
    //      joint1:
    //        offset: 0.01
    //        range: [0.,1.]
    //        max_vel: 0.03
    //        near_tolerance: 0.03
    offset_ = xmlRpcGetDouble(joint, "offset", 0.);
    max_vel_ = xmlRpcGetDouble(joint, "max_vel", 1.0);
    near_tolerance_ = xmlRpcGetDouble(joint, "near_tolerance", 0.05);
    ROS_ASSERT(joint["range"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    min_position_ = xmlRpcGetDouble(joint["range"], 0);
    max_position_ = xmlRpcGetDouble(joint["range"], 1);
  }
  bool judgeJointPosition()
  {
    return abs(current_position_ - offset_ - min_position_) <= near_tolerance_ ||
           abs(max_position_ + offset_ - current_position_) <= near_tolerance_;
  }
  int judgeMoveDirect()
  {
    return ((current_position_ - offset_) >= ((max_position_ - min_position_) / 2)) ? 1 : -1;
  }

private:
  double offset_, max_position_, min_position_, current_position_, max_vel_, near_tolerance_;
};

class ProgressBase
{
public:
  ProgressBase(XmlRpc::XmlRpcValue& progress)
  {
    // progress:
    //        timeout: 3.
    time_out_ = xmlRpcGetDouble(progress, "timeout", 1e10);
  }
  virtual void nextProcess(bool flag);
  virtual void manageProcess(bool flag);
  virtual void run();
  virtual void printProcess();
  bool checkTimeout(ros::Duration period)
  {
    if (period.toSec() > time_out_)
    {
      ROS_ERROR("Step timeout,it should be finish in %f seconds", time_out_);
      return false;
    }
    return true;
  }

protected:
  int process_;
  bool is_finish_{ false };
  double time_out_{};
  ros::Time start_time_;
};

class Find : public ProgressBase
{
public:
  Find(XmlRpc::XmlRpcValue& find) : ProgressBase(find)
  {
    //        find:
    //          pitch:
    //        offset: 0.01
    //        range: [0.,1.]
    //        max_vel: 0.03
    //        near_tolerance: 0.03
    //          confirm_lock_time
    process_ = SWING;
    ROS_ASSERT(find["yaw"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(find["yaw"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    yaw_ = new JointInfo(find["yaw"]);
    pitch_ = new JointInfo(find["pitch_"]);
    confirm_lock_time_ = xmlRpcGetDouble(find, "confirm_lock_time", 10);
  }
  void nextProcess(bool flag) override
  {
    process_++;
    printProcess();
  }
  void manageProcess(bool flag) override
  {
    if (process_ != LOCKED)
      nextProcess(flag);
    else
      is_finish_ = true;
  }
  void printProcess() override
  {
    if (process_ == SWING)
      ROS_INFO_STREAM("SWING");
    else if (process_ == FOUND)
      ROS_INFO_STREAM("FOUND");
    else if (process_ == LOCKED)
      ROS_INFO_STREAM("LOCKED");
  }
  void run() override
  {
  }

private:
  JointInfo *yaw_{}, *pitch_{};
  double search_angle_{}, confirm_lock_time_{};
};

class ProAdjust
{
};

class ServoMove
{
};

class MotionMove
{
};

class PostAdjust
{
};

class Finish
{
};

}  // namespace auto_exchange
