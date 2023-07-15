//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>

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
    //        near_tolerance: 0.03
    offset_ = xmlRpcGetDouble(joint, "offset", 0.);
    near_tolerance_ = xmlRpcGetDouble(joint, "near_tolerance", 0.05);
    ROS_ASSERT(joint["range"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    min_position_ = xmlRpcGetDouble(joint["range"], 0);
    max_position_ = xmlRpcGetDouble(joint["range"], 1);
  }
  bool judgeJointPosition()
  {
    return (abs(current_position_ - offset_ - min_position_) <= near_tolerance_ ||
            abs(max_position_ + offset_ - current_position_) <= near_tolerance_) ?
               true :
               false;
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
    time_out_ = xmlRpcGetDouble(progress["common"], "timeout", 1e10);
  }
  virtual void nextProcess();
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

protected:
  double time_out_{};
};

class Find
{
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
