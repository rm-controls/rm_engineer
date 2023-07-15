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
  }
  bool judgeJointPosition()
  {
    return (abs(current_position - offset - min_position) <= near_tolerance_ ||
            abs(max_position + offset - current_position) <= near_tolerance_) ?
               true :
               false;
  }

private:
  int move_direct;
  double offset, max_position, min_position, current_position, max_vel, near_tolerance_;
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
