//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/ExchangerMsg.h>
#include <control_toolbox/pid.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

namespace auto_exchange
{
class JointInfo
{
public:
  JointInfo(XmlRpc::XmlRpcValue& joint)
  {
    offset_ = xmlRpcGetDouble(joint, "offset", 0.);
    max_scale_ = xmlRpcGetDouble(joint, "max_scale", 1.0);
    near_tolerance_ = xmlRpcGetDouble(joint, "near_tolerance", 0.05);
    ROS_ASSERT(joint["range"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    min_position_ = xmlRpcGetDouble(joint["range"], 0);
    max_position_ = xmlRpcGetDouble(joint["range"], 1);
    move_direct_ = -1;
  }
  bool judgeJointLimit()
  {
    return abs(current_position_ - offset_ - min_position_) <= near_tolerance_ ||
           abs(max_position_ + offset_ - current_position_) <= near_tolerance_;
  }
  void judgeMoveDirect()
  {
    if (current_position_ - offset_ <= min_position_)
      move_direct_ = 1;
    else if (current_position_ - offset_ >= max_position_)
      move_direct_ = -1;
  }

public:
  int move_direct_;
  double offset_, max_position_, min_position_, current_position_, max_scale_, near_tolerance_;
};

class SingleDirectionMove
{
public:
  std::string name;
  double tolerance, start_vel, offset_refer_exchanger, max_vel, error, pid_value;
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
  bool isFinish()
  {
    return abs(error) <= tolerance;
  }
  double computerVel(ros::Duration dt)
  {
    double vel = start_vel + abs(pid.computeCommand(error, dt));
    int direction = error / abs(error);
    return abs(vel) >= max_vel ? direction * max_vel : direction * vel;
  }
  void getPidValue(ros::Duration dt)
  {
    double vel = start_vel + abs(pid.computeCommand(error, dt));
    int direction = error / abs(error);
    pid_value = abs(vel) >= max_vel ? direction * max_vel : direction * vel;
  }
};

class ProgressBase
{
public:
  ProgressBase(XmlRpc::XmlRpcValue& progress, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : tf_buffer_(tf_buffer), nh_(nh)
  {
    time_out_ = xmlRpcGetDouble(progress, "time_out", 1e10);
    if (nh_.hasParam("internal_time_out"))
    {
      internal_time_out_.resize((int)progress["internal_time_out"].size(), 1.);
      for (int i = 0; i < (int)progress["internal_time_out"].size(); ++i)
        internal_time_out_[i] = (progress["internal_time_out"][i]);
    }
  }
  virtual void init()
  {
    enter_flag_ = false;
    is_finish_ = false;
    is_recorded_time_ = false;
    is_recorded_internal_time_ = false;
  }
  virtual void stateMachine() = 0;
  virtual void run()
  {
    enter_flag_ = true;
    if (!is_finish_)
    {
      checkTimeout();
      if (nh_.hasParam("internal_time_out"))
      {
        checkInternalTimeout();
      }
      stateMachine();
    }
  }
  virtual void printProcess() = 0;
  bool getFinishFlag()
  {
    return is_finish_;
  }
  bool getEnterFlag()
  {
    return enter_flag_;
  }
  bool getTimeOutFlag()
  {
    return is_time_out_;
  }
  bool getProcess()
  {
    return process_;
  }
  void checkTimeout()
  {
    if (!is_recorded_time_)
    {
      is_recorded_time_ = true;
      start_time_ = ros::Time::now();
    }
    if ((ros::Time::now() - start_time_).toSec() > time_out_)
    {
      ROS_ERROR("Progress timeout, should be finish in %f seconds", time_out_);
      is_recorded_time_ = false;
      is_finish_ = true;
      is_time_out_ = true;
    }
  }
  void checkInternalTimeout()
  {
    if (!is_recorded_internal_time_ || last_process_ != process_)
    {
      is_recorded_internal_time_ = true;
      internal_start_time_ = ros::Time::now();
      last_process_ = process_;
    }
    if ((ros::Time::now() - internal_start_time_).toSec() > internal_time_out_[process_])
    {
      ROS_ERROR("Inside progress timeout, should be finish in %f seconds", internal_time_out_[process_]);
      //      ROS_INFO_STREAM("dt:  " <<  (ros::Time::now() - internal_start_time_).toSec());
      is_recorded_internal_time_ = false;
      if (process_ < (process_num_ - 1))
        process_++;
      else
        is_finish_ = true;
    }
  }

protected:
  tf2_ros::Buffer& tf_buffer_;
  int process_{}, last_process_{}, process_num_{};
  bool is_finish_{ false }, is_recorded_time_{ false }, enter_flag_{ false }, is_recorded_internal_time_{ false },
      is_time_out_{ false };
  double time_out_{};
  std::vector<double> internal_time_out_{};
  ros::Time start_time_{}, internal_start_time_{};
  ros::NodeHandle nh_{};
};

class Find : public ProgressBase
{
public:
  enum FindProcess
  {
    SWING,
    ADJUST,
    FINISH
  };
  Find(XmlRpc::XmlRpcValue& find, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh) : ProgressBase(find, tf_buffer, nh)
  {
    process_ = SWING;
    last_process_ = process_;
    process_num_ = 3;
    gimbal_scale_.resize(2, 0);
    chassis_scale_.resize(2, 0);
    search_range_ = xmlRpcGetDouble(find, "search_range", 0.3);
    ROS_ASSERT(find["yaw"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(find["pitch"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    yaw_ = new JointInfo(find["yaw"]);
    pitch_ = new JointInfo(find["pitch"]);
    visual_recognition_sub_ =
        nh_.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 1, &Find::visualRecognitionCallback, this);
    ROS_INFO_STREAM("~~~~~~~~~~~~~FIND~~~~~~~~~~~~~~~~");
  }
  void init() override
  {
    ProgressBase::init();
    process_ = { SWING };
    initScales();
  }
  std::vector<double> getGimbalScale()
  {
    return gimbal_scale_;
  }
  std::vector<double> getChassisScale()
  {
    return chassis_scale_;
  }
  void testAdjust()
  {
    gimbal_scale_[0] = middle_point_.x / 200;
    gimbal_scale_[1] = middle_point_.y / 200;
  }

private:
  void stateMachine() override
  {
    switch (process_)
    {
      case SWING:
      {
        autoSearch(false, true);
        if (is_found_)
        {
          //            process_ = ADJUST;
          process_ = FINISH;
        }
      }
      break;
      case ADJUST:
      {
        if (is_found_)
        {
          gimbal_scale_[0] = middle_point_.x / abs(middle_point_.x) * yaw_->max_scale_;
          gimbal_scale_[1] = middle_point_.y / abs(middle_point_.y) * pitch_->max_scale_;
          if (sqrt(pow(middle_point_.x, 2) + pow(middle_point_.y, 2)) < 200)
            process_ = FINISH;
        }
        else
          process_ = SWING;
      }
      break;
      case FINISH:
      {
        is_finish_ = true;
        ROS_INFO_STREAM("LOCKED");
      }
      break;
    }
  }

  void autoSearch(bool enable_chassis, bool enable_gimbal)
  {
    if (enable_gimbal)
    {
      geometry_msgs::TransformStamped base2yaw, yaw2pitch;
      base2yaw = tf_buffer_.lookupTransform("gimbal_base", "yaw", ros::Time(0));
      yaw2pitch = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0));

      double yaw = yawFromQuat(base2yaw.transform.rotation);
      double roll_temp, pitch, yaw_temp;
      quatToRPY(yaw2pitch.transform.rotation, roll_temp, pitch, yaw_temp);
      yaw_->current_position_ = yaw / search_range_;
      pitch_->current_position_ = pitch / search_range_;
      yaw_->judgeMoveDirect();
      pitch_->judgeMoveDirect();
      gimbal_scale_[0] = yaw_->move_direct_ * yaw_->max_scale_;
      gimbal_scale_[1] = pitch_->move_direct_ * pitch_->max_scale_;
    }
    if (enable_chassis)
    {
      chassis_scale_[0] = gimbal_scale_[0];
      chassis_scale_[1] = gimbal_scale_[1];
    }
  }
  void printProcess() override
  {
    if (process_ == SWING)
      ROS_INFO_STREAM("SWING");
    else if (process_ == ADJUST)
      ROS_INFO_STREAM("FOUND");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
  }
  void initScales()
  {
    for (int i = 0; i < (int)gimbal_scale_.size(); ++i)
    {
      gimbal_scale_[i] = 0;
      chassis_scale_[i] = 0;
    }
  }
  void visualRecognitionCallback(const rm_msgs::ExchangerMsg ::ConstPtr& msg)
  {
    is_found_ = msg->flag;
    middle_point_ = msg->middle_point;
  }
  JointInfo *yaw_{}, *pitch_{};
  bool is_found_{ false };
  geometry_msgs::Point middle_point_;
  std::vector<double> gimbal_scale_{}, chassis_scale_{};
  ros::Subscriber visual_recognition_sub_{};
  double search_range_{};
};

class ProAdjust : public ProgressBase
{
public:
  enum AdjustProcess
  {
    SET_GOAL,
    CHASSIS_X,
    CHASSIS_Y,
    CHASSIS_YAW,
    FINISH
  };
  ProAdjust(XmlRpc::XmlRpcValue& pre_adjust, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(pre_adjust, tf_buffer, nh)
  {
    process_ = SET_GOAL;
    last_process_ = process_;
    process_num_ = 5;
    x_.init(pre_adjust["x"], "x", nh);
    y_.init(pre_adjust["y"], "y", nh);
    yaw_.init(pre_adjust["yaw"], "yaw", nh);
    ROS_INFO_STREAM("~~~~~~~~~~~~~PRE_ADJUST~~~~~~~~~~~~~~~~");
  }
  void init() override
  {
    ProgressBase::init();
    process_ = SET_GOAL;
    re_adjust_ = false;
    initComputerValue();
  }
  void stateMachine() override
  {
    switch (process_)
    {
      case SET_GOAL:
      {
        set_goal();
        process_ = CHASSIS_X;
      }
      break;
      case CHASSIS_X:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.y = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (x_.isFinish())
          process_ = re_adjust_ ? FINISH : CHASSIS_Y;
      }
      break;
      case CHASSIS_Y:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (y_.isFinish())
          process_ = CHASSIS_YAW;
      }
      break;
      case CHASSIS_YAW:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.linear.y = 0.;
        if (yaw_.isFinish())
        {
          process_ = SET_GOAL;
          re_adjust_ = true;
        }
      }
      break;
      case FINISH:
      {
        is_finish_ = true;
        re_adjust_ = false;
        initComputerValue();
        ROS_INFO_STREAM("PRE ADJUST FINISH");
      }
      break;
    }
  }
  geometry_msgs::Twist getChassisVelMsg()
  {
    return chassis_vel_cmd_;
  }
  std::string getChassisCmdFrame()
  {
    return chassis_command_source_frame_;
  }

private:
  void initComputerValue()
  {
    chassis_vel_cmd_.linear.x = 0;
    chassis_vel_cmd_.linear.y = 0;
    chassis_vel_cmd_.linear.z = 0;
    chassis_vel_cmd_.angular.x = 0;
    chassis_vel_cmd_.angular.y = 0;
    chassis_vel_cmd_.angular.z = 0;
  }
  void printProcess() override
  {
    ROS_INFO_STREAM("PRE ADJUST");
  }
  void computerChassisVel()
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
  void set_goal()
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

  bool re_adjust_{ false };
  ros::Time last_time_;
  std::string chassis_command_source_frame_{ "base_link" };
  geometry_msgs::Twist chassis_vel_cmd_{};
  geometry_msgs::PoseStamped chassis_target_{}, chassis_original_target_{};
  SingleDirectionMove x_, y_, yaw_;
};

class AutoServoMove : public ProgressBase
{
public:
  enum ServoMoveProcess
  {
    YZ,
    YAW,
    ROLL,
    REZ,
    PITCH,
    REY,
    RREZ,
    PUSH,
    FINISH
  };
  AutoServoMove(XmlRpc::XmlRpcValue& auto_servo_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_servo_move, tf_buffer, nh), joint7_msg_(0.)
  {
    process_ = YZ;
    last_process_ = process_;
    process_num_ = 6;

    x_.init(auto_servo_move["x"], "x", nh);
    y_.init(auto_servo_move["y"], "y", nh);
    z_.init(auto_servo_move["z"], "z", nh);
    re_y_.init(auto_servo_move["re_y"], "re_y", nh);
    re_z_.init(auto_servo_move["re_z"], "re_z", nh);
    roll_.init(auto_servo_move["roll"], "roll", nh);
    pitch_.init(auto_servo_move["pitch"], "pitch", nh);
    yaw_.init(auto_servo_move["yaw"], "yaw", nh);
    move_gather_.clear();
    move_gather_.push_back(x_);
    move_gather_.push_back(y_);
    move_gather_.push_back(z_);
    move_gather_.push_back(roll_);
    move_gather_.push_back(pitch_);
    move_gather_.push_back(yaw_);
    move_gather_.push_back(re_y_);
    move_gather_.push_back(re_z_);

    servo_pid_value_.resize(6, 0.);
    servo_scales_.resize(6, 0.);
    ROS_INFO_STREAM("~~~~~~~~~~~~~SERVO_MOVE~~~~~~~~~~~~~~~~");
  }
  ~AutoServoMove() = default;
  void init() override
  {
    ProgressBase::init();
    process_ = { YZ };
    initComputerValue();
    joint7_msg_ = 0.;
  }
  double getJoint7Msg()
  {
    return joint7_msg_;
  }
  std::vector<double> getServoScale()
  {
    return servo_scales_;
  }

  void printProcess() override
  {
    if (process_ == YZ)
      ROS_INFO_STREAM("YZ");
    else if (process_ == YAW)
      ROS_INFO_STREAM("YAW");
    else if (process_ == ROLL)
      ROS_INFO_STREAM("ROLL");
    else if (process_ == PITCH)
      ROS_INFO_STREAM("PITCH");
    else if (process_ == REY)
      ROS_INFO_STREAM("REY");
    else if (process_ == REZ)
      ROS_INFO_STREAM("REZ");
    else if (process_ == PUSH)
      ROS_INFO_STREAM("PUSH");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
  }

private:
  void stateMachine() override
  {
    computeServoMoveScale();
    manageServoMoveProcess();
  }
  void initComputerValue()
  {
    for (int i = 0; i < (int)servo_pid_value_.size(); ++i)
    {
      servo_pid_value_[i] = 0.;
      servo_scales_[i] = 0.;
    }
    move_gather_[0] = x_;
    move_gather_[1] = y_;
    move_gather_[2] = z_;
    move_gather_[3] = roll_;
    move_gather_[4] = pitch_;
    move_gather_[5] = yaw_;
    move_gather_[6] = re_y_;
    move_gather_[7] = re_z_;
  }
  void computeServoMoveError()
  {
    initComputerValue();
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
    x_.error = (process_ == PUSH ? (tools2exchanger.transform.translation.x) :
                                   (tools2exchanger.transform.translation.x - x_.offset_refer_exchanger));
    y_.error = tools2exchanger.transform.translation.y - y_.offset_refer_exchanger;
    re_y_.error = y_.error;
    z_.error = tools2exchanger.transform.translation.z - z_.offset_refer_exchanger;
    re_z_.error = z_.error;
    roll_.error = roll * roll_.offset_refer_exchanger;
    pitch_.error = pitch * pitch_.offset_refer_exchanger;
    yaw_.error = yaw * yaw_.offset_refer_exchanger;

    ros::Duration dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();

    servo_pid_value_[0] = x_.computerVel(dt);
    if (process_ != REY)
      servo_pid_value_[1] = y_.computerVel(dt);
    else
      servo_pid_value_[1] = re_y_.computerVel(dt);
    if (process_ == REZ || process_ == RREZ)
      servo_pid_value_[2] = re_z_.computerVel(dt);
    else
      servo_pid_value_[2] = z_.computerVel(dt);
    servo_pid_value_[3] = roll_.computerVel(dt);
    servo_pid_value_[4] = pitch_.computerVel(dt);
    servo_pid_value_[5] = yaw_.computerVel(dt);
  }
  void computeServoMoveScale()
  {
    computeServoMoveError();
    switch (process_)
    {
      case YZ:
      {
        servo_scales_[1] = servo_pid_value_[1];
        servo_scales_[2] = servo_pid_value_[2];
      }
      break;
      case YAW:
      {
        servo_scales_[5] = servo_pid_value_[5];
      }
      break;
      case ROLL:
      {
        servo_scales_[3] = servo_pid_value_[3];
      }
      break;
      case REZ:
      {
        servo_scales_[2] = servo_pid_value_[2];
      }
      break;
      case PITCH:
      {
        joint7_msg_ = pitch_.error;
      }
      break;
      case REY:
      {
        servo_scales_[1] = servo_pid_value_[1];
      }
      break;
      case RREZ:
      {
        servo_scales_[2] = servo_pid_value_[2];
      }
      break;
      case PUSH:
      {
        servo_scales_[0] = servo_pid_value_[0];
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
        if (process_ == YZ)
        {
          if (abs(move_gather_[i].error) <= (y_.tolerance + z_.tolerance) / 2)
            arrived_joint_num++;
        }
        else if (process_ == REZ)
        {
          if (abs(re_z_.error) <= re_z_.tolerance)
            arrived_joint_num++;
        }
        else if (process_ == REY)
        {
          if (abs(re_y_.error) <= re_y_.tolerance)
            arrived_joint_num++;
        }
        else
        {
          if (abs(move_gather_[i].error) <= move_gather_[i].tolerance)
            arrived_joint_num++;
        }
      }
    }
    if (arrived_joint_num == move_joint_num)
    {
      if (process_ != FINISH)
        process_++;
      else
        is_finish_ = true;
    }
  }
  void rectifyForLink7(double theta, double link7_length)
  {
    rectify_x_ = link7_length_ * pow(sin(theta), 2) / (tan(M_PI_2 - theta / 2));
    rectify_z_ = link7_length_ * sin(theta) * cos(theta) / (tan(M_PI_2 - theta / 2));
  }

  SingleDirectionMove x_, y_, z_, roll_, pitch_, yaw_, re_y_, re_z_;
  std::vector<SingleDirectionMove> move_gather_{};
  ros::Time last_time_;
  double link7_length_{}, joint7_msg_{}, rectify_x_, rectify_z_;
  std::vector<double> servo_errors_{}, servo_scales_{}, servo_pid_value_{};
};

class UnionMove : public ProgressBase
{
public:
  enum UnionMoveProcess
  {
    MOTION,
    SERVO_Z,
    SERVO_Y,
    SERVO_X,
    FINISH
  };
  UnionMove(XmlRpc::XmlRpcValue& union_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(union_move, tf_buffer, nh)
  {
    process_ = MOTION;
    last_process_ = process_;
    process_num_ = 4;
    motion_name_ = union_move.hasMember("motion_name") ? std::string(union_move["motion_name"]) : "AUTO_EXCHANGE";
    x_.init(union_move["x"], "x", nh);
    y_.init(union_move["y"], "y", nh);
    z_.init(union_move["z"], "z", nh);
    move_gather_.clear();
    move_gather_.push_back(x_);
    move_gather_.push_back(y_);
    move_gather_.push_back(z_);
    servo_scales_.resize(3, 0.);
  }
  std::string getMotionName()
  {
    return motion_name_;
  }
  bool getIsMotionFinish()
  {
    return is_motion_finish_;
  }
  bool getIsMotionStart()
  {
    return is_motion_start_;
  }
  void changeIsMotionFinish(bool state)
  {
    is_motion_finish_ = state;
  }
  void changeIsMotionStart(bool state)
  {
    is_motion_start_ = state;
  }
  std::vector<double> getServoScale()
  {
    return servo_scales_;
  }
  void init() override
  {
    ProgressBase::init();
    initComputerValue();
    process_ = MOTION;
    is_motion_start_ = false;
    is_motion_finish_ = false;
  }

private:
  void printProcess() override
  {
    ROS_INFO_STREAM(process_);
  }
  void stateMachine() override
  {
    if (is_motion_finish_)
      process_ = SERVO_Z;
    if (process_ != MOTION)
    {
      computeServoMoveScale();
      manageServoMoveProcess();
    }
  }
  void initComputerValue()
  {
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      servo_scales_[i] = 0.;
    }
    move_gather_[0] = x_;
    move_gather_[1] = y_;
    move_gather_[2] = z_;
  }
  void computeServoMoveError()
  {
    initComputerValue();
    geometry_msgs::TransformStamped tools2exchanger;
    try
    {
      tools2exchanger = tf_buffer_.lookupTransform("tools_link", "exchanger", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    x_.error = tools2exchanger.transform.translation.x - x_.offset_refer_exchanger;
    y_.error = tools2exchanger.transform.translation.y - y_.offset_refer_exchanger;
    z_.error = tools2exchanger.transform.translation.z - z_.offset_refer_exchanger;

    ros::Duration dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();

    x_.getPidValue(dt);
    y_.getPidValue(dt);
    z_.getPidValue(dt);
  }
  void computeServoMoveScale()
  {
    computeServoMoveError();
    switch (process_)
    {
      case SERVO_Z:
      {
        servo_scales_[0] = 0.;
        servo_scales_[1] = 0.;
        servo_scales_[2] = z_.pid_value;
      }
      break;
      case SERVO_Y:
      {
        servo_scales_[0] = 0.;
        servo_scales_[1] = y_.pid_value;
        servo_scales_[2] = 0.;
      }
      break;
      case SERVO_X:
      {
        servo_scales_[0] = x_.pid_value;
        servo_scales_[1] = 0.;
        servo_scales_[2] = 0.;
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
        if (abs(move_gather_[i].error) <= move_gather_[i].tolerance)
          arrived_joint_num++;
      }
    }
    if (arrived_joint_num == move_joint_num)
    {
      if (process_ != FINISH)
        process_++;
      else
        is_finish_ = true;
    }
  }
  bool is_motion_finish_{ false }, is_motion_start_{ false };
  ros::Time last_time_;
  SingleDirectionMove x_, y_, z_;
  std::string motion_name_;
  std::vector<double> servo_scales_{};
  std::vector<SingleDirectionMove> move_gather_{};
};

// class MotionMove : public ProgressBase
//{
// public:
//     enum MotionMoveProcess
//     {
//         SPHERE,
//         LINE,
//         POINT,
//         FINISH
//     };
//     MotionMove(XmlRpc::XmlRpcValue& auto_servo_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
//     : ProgressBase(auto_servo_move, tf_buffer, nh) {
//         process_ = SPHERE;
//         last_process_ = process_;
//         process_num_ = 4;
//     }
//
//};
//
// class PostAdjust
//{
//};
//
// class Finish
//{
//};

class AutoExchange : public ProgressBase
{
public:
  enum ExchangeProcess
  {
    FIND,
    PRE_ADJUST,
    MOVE,
    FINISH
  };
  AutoExchange(XmlRpc::XmlRpcValue& auto_exchange, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_exchange, tf_buffer, nh)
  {
    process_ = FIND;
    last_process_ = process_;
    process_num_ = 4;
    ros::NodeHandle nh_auto_find(nh, "auto_find");
    ros::NodeHandle nh_auto_pre_adjust(nh, "auto_pre_adjust");
    ros::NodeHandle nh_auto_servo_move(nh, "auto_servo_move");
    ros::NodeHandle nh_union_move(nh, "union_move");
    find_ = new Find(auto_exchange["auto_find"], tf_buffer, nh_auto_find);
    pre_adjust_ = new ProAdjust(auto_exchange["auto_pre_adjust"], tf_buffer, nh_auto_pre_adjust);
    auto_servo_move_ = new AutoServoMove(auto_exchange["auto_servo_move"], tf_buffer, nh_auto_servo_move);
    union_move_ = new UnionMove(auto_exchange["union_move"], tf_buffer, nh_union_move);
    exchanger_tf_update_pub_ = nh_.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
  }
  void init() override
  {
    ProgressBase::init();
    process_ = FIND;
    find_->init();
    pre_adjust_->init();
    union_move_->init();
    exchangerTfUpdate(true);
  }

public:
  Find* find_{};
  ProAdjust* pre_adjust_{};
  AutoServoMove* auto_servo_move_{};
  UnionMove* union_move_{};

private:
  void exchangerTfUpdate(bool is_exchanger_tf_update)
  {
    is_exchanger_tf_update_.data = is_exchanger_tf_update;
    exchanger_tf_update_pub_.publish(is_exchanger_tf_update_);
  }
  void stateMachine() override
  {
    switch (process_)
    {
      case FIND:
      {
        exchangerTfUpdate(true);
        find_->run();
        if (find_->getFinishFlag())
        {
          if (find_->getTimeOutFlag() && find_->getProcess() == 0)
          {
            is_finish_ = true;
            find_->init();
          }
          else
          {
            process_ = PRE_ADJUST;
            find_->init();
          }
        }
      }
      break;
      case PRE_ADJUST:
      {
        exchangerTfUpdate(true);
        pre_adjust_->run();
        if (pre_adjust_->getFinishFlag())
        {
          process_ = MOVE;
          pre_adjust_->init();
        }
      }
      break;
      case MOVE:
      {
        exchangerTfUpdate(false);
        union_move_->run();
        if (union_move_->getFinishFlag())
        {
          is_finish_ = true;
          union_move_->init();
        }
      }
      break;
    }
  }
  void printProcess() override
  {
    if (process_ == FIND)
      ROS_INFO_STREAM("FIND");
    else if (process_ == PRE_ADJUST)
      ROS_INFO_STREAM("PRE_ADJUST");
    else if (process_ == MOVE)
      ROS_INFO_STREAM("MOVE");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
  }
  // tf update
  std_msgs::Bool is_exchanger_tf_update_{};
  ros::Publisher exchanger_tf_update_pub_;
};
}  // namespace auto_exchange
