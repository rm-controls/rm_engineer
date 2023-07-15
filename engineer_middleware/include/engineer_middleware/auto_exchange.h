//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rm_common/ori_tool.h>

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
  ProgressBase(XmlRpc::XmlRpcValue& progress, tf2_ros::Buffer& tf_buffer) : tf_buffer_(tf_buffer)
  {
    // progress:
    //        timeout: 3.
    time_out_ = xmlRpcGetDouble(progress, "timeout", 1e10);
  }
  virtual void init();
  virtual void nextProcess();
  virtual void manageProcess();
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
  tf2_ros::Buffer& tf_buffer_;
  int process_;
  bool is_finish_{ false };
  double time_out_{};
  ros::Time start_time_;
};

class Find : public ProgressBase
{
public:
  Find(XmlRpc::XmlRpcValue& find, tf2_ros::Buffer& tf_buffer) : ProgressBase(find, tf_buffer)
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
  void init() override
  {
    is_finish_ = false;
    process_ = { SWING };
  }
  void nextProcess() override
  {
    process_++;
    printProcess();
  }
  void manageProcess() override
  {
    if (process_ != LOCKED)
      nextProcess();
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

class ServoMove : public ProgressBase
{
public:
  ServoMove(XmlRpc::XmlRpcValue& servo_move, ros::NodeHandle& nh, tf2_ros::Buffer& tf_buffer)
    : ProgressBase(servo_move, tf_buffer)
  {
    //      servo_move:
    //        xyz_offset: [ 0.08, 0., -0.04]
    //        link7_length: 0.1
    //        servo_p: [ 6., 6., 7., 3., 2., 1. ]
    //        servo_error_tolerance: [ 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 ]
    ROS_ASSERT(servo_move["xyz_offset"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(servo_move["servo_p"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(servo_move["servo_error_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(servo_move["link7_length"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    process_ = { YZ };
    enter_servo_move_.data = false;
    xyz_offset_.resize(3, 0.);
    servo_p_.resize(6, 0.);
    servo_errors_.resize(6, 0.);
    servo_scales_.resize(6, 0.);
    servo_error_tolerance_.resize(6, 0.01);
    for (int i = 0; i < (int)xyz_offset_.size(); ++i)
      xyz_offset_[i] = servo_move["xyz_offset"][i];
    for (int i = 0; i < (int)servo_p_.size(); ++i)
    {
      servo_p_[i] = servo_move["servo_p"][i];
      servo_error_tolerance_[i] = servo_move["servo_error_tolerance"][i];
    }
    exchanger_tf_update_pub_ = nh.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
  }
  void init() override
  {
    is_finish_ = false;
    process_ = { YZ };
    enter_servo_move_.data = false;
    initComputerValue();
  }

  void nextProcess() override
  {
    process_++;
    printProcess();
  }
  void manageProcess() override
  {
    if (process_ != DONE)
      nextProcess();
    else
      is_finish_ = true;
  }
  void printProcess() override
  {
    if (process_ == YZ)
      ROS_INFO_STREAM("YZ");
    else if (process_ == ROLL_YAW)
      ROS_INFO_STREAM("ROLL_YAW");
    else if (process_ == XYZ)
      ROS_INFO_STREAM("XYZ");
    else if (process_ == PITCH)
      ROS_INFO_STREAM("PITCH");
    else if (process_ == PUSH)
      ROS_INFO_STREAM("PUSH");
  }
  void run() override
  {
    enter_servo_move_.data = true;
    exchanger_tf_update_pub_.publish(enter_servo_move_);
    if (!is_finish_)
    {
      computeServoMoveScale();
      manageProcess();
    }
  }

private:
  void initComputerValue()
  {
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      servo_errors_[i] = 0;
      servo_scales_[i] = 0;
    }
  }
  void computeServoMoveError()
  {
    double roll, pitch, yaw;
    std::vector<double> errors;
    geometry_msgs::TransformStamped tools2exchanger;
    try
    {
      tools2exchanger = tf_buffer_.lookupTransform("tools_link", "exchanger", ros::Time(0));
      quatToRPY(tools2exchanger.transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    initComputerValue();
    servo_errors_[0] = (process_ == PUSH ? (tools2exchanger.transform.translation.x) :
                                           (tools2exchanger.transform.translation.x - xyz_offset_[0]));
    servo_errors_[1] = tools2exchanger.transform.translation.y - xyz_offset_[1] + 0.06 * sin(roll + M_PI_2);
    servo_errors_[2] = tools2exchanger.transform.translation.z - xyz_offset_[2];
    servo_errors_[3] = roll;
    servo_errors_[4] = pitch;
    servo_errors_[5] = yaw;
  }
  void computeServoMoveScale(rm_common::JointPositionBinaryCommandSender* joint7_command_sender)
  {
    computeServoMoveError();
    switch (process_)
    {
      case YZ:
      {
        for (int i = 1; i < 3; ++i)
        {
          servo_scales_[i] = servo_errors_[i] * servo_p_[i];
        }
      }
      break;
      case ROLL_YAW:
      {
        servo_scales_[3] = servo_errors_[3] * servo_p_[3];
        servo_scales_[5] = servo_errors_[5] * servo_p_[5];
      }
      break;
      case XYZ:
      {
        for (int i = 0; i < 3; ++i)
          servo_scales_[i] = servo_errors_[i] * servo_p_[i];
      }
      break;
      case PITCH:
      {
        joint7_command_sender->getMsg()->data = servo_errors_[4];
      }
      break;
      case PUSH:
      {
        servo_scales_[0] = servo_errors_[0] * servo_p_[0];
      }
      break;
    }
  }
  void manageServoMoveProcess()
  {
    int move_joint_num = 0, arrived_joint_num = 0;
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      if (servo_scales_[i] != 0)
      {
        move_joint_num++;
        if (abs(servo_errors_[i]) <= servo_error_tolerance_[i])
          arrived_joint_num++;
      }
    }
    if (arrived_joint_num == move_joint_num)
      nextProcess();
    //        if (checkTimeout())
    //        {
    //            switch ()
    //            {
    //            }
    //            nextProcess();
    //            ROS_INFO_STREAM("TIME OUT");
    //        }
    //        else if (arrived_joint_num == move_joint_num)
    //            nextProcess();
  }
  std_msgs::Bool enter_servo_move_{};
  double link7_length_{};
  ros::Publisher exchanger_tf_update_pub_;
  std::vector<double> xyz_offset_{}, servo_p_{}, servo_errors_{}, servo_scales_{}, servo_error_tolerance_{};
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
