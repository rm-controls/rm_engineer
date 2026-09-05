//
// Created by ch on 25-7-6.
//
#include "engineer_trajectory_teaching/engineer_trajectory_teaching.h"

namespace engineer_trajectory_teaching
{
  EngineerTrajectoryTeaching::EngineerTrajectoryTeaching(ros::NodeHandle &nh)
  : nh_(nh),
    as_(nh_, "trajectory_server", [this](auto&& PH1) { executeCB(std::forward<decltype(PH1)>(PH1)); }, false)
  {
    ROS_INFO_STREAM(nh_.getNamespace().c_str());
    ROS_ASSERT(nh_.getParam("record/record_joint_names", recorded_trajectory_.record_joint_names));
    nh_.param("record/record_interval_threshold", record_interval_threshold_, 0.1);
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &EngineerTrajectoryTeaching::jointStateCallback, this);
    controller_state_sub_ = nh_.subscribe("/controllers/arm_trajectory_controller/state", 10, &EngineerTrajectoryTeaching::controllerStateCallback, this);
    trajectory_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/controllers/arm_trajectory_controller/command", 10);
    small_joint1_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/controllers/small_joint1_controller/command", 10);
    small_joint2_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/controllers/small_joint2_controller/command", 10);
    small_joint3_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/controllers/small_joint3_controller/command", 10);
    gpio_cmd_pub_ = nh_.advertise<rm_msgs::GpioData>("/controllers/gpio_controller/command", 10);
    gimbal_cmd_pub_ = nh_.advertise<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10);
    middle_pitch_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/controllers/middle_pitch_controller/command", 10);
    as_.start();
  }

  void EngineerTrajectoryTeaching::executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerTrajectoryTeachingAction>::GoalConstPtr &goal)
  {
    result_.succeed = false;
    if (goal->filename.empty())
    {
      ROS_ERROR_STREAM("The file path is empty!");
      as_.setAborted(result_);
      return;
    }
    file_path_ = goal->filename;
    if (goal->command == rm_msgs::EngineerTrajectoryTeachingGoal_<std::allocator<void>>::RECORD)
    {
      startRecord();
      while (as_.isActive())
      {
        record();
        if (as_.isPreemptRequested() || !ros::ok())
        {
          stopRecord(file_path_);
          break;
        }
      }
      result_.succeed = true;
      as_.setSucceeded(result_);
    }
    if (goal->command == rm_msgs::EngineerTrajectoryTeachingGoal_<std::allocator<void>>::PLAYBACK)
    {
      if (goal->sync_time <= 0.0 || goal->speed <= 0.0)
      {
        ROS_ERROR_STREAM("THe sync_time or speed is invalid!");
        as_.setAborted(result_);
        return;
      }
      sync_time_ = goal->sync_time;
      speed_ = goal->speed;
      try
      {
        YAML::Node config = YAML::LoadFile(file_path_);
        ros::Time last_time = ros::Time::now();

        ros::Duration duration = ros::Duration(0.0);
        auto joint_names = config["joint_names"].as<std::vector<std::string>>();
        trajectory_cmd_.joint_names = joint_names;
        auto points = config["points"];
        auto point = points.begin();
        auto last_time_stamp = (*point)["timestamp"].as<double>();
        while (as_.isActive() && point != points.end())
        {
          if (ros::Time::now() - last_time >= duration)
          {
            duration = ros::Duration(((*point)["timestamp"].as<double>() - last_time_stamp) / speed_);
            if (point == points.begin())
              duration += ros::Duration(sync_time_);
            playback(*point, duration);
            last_time_stamp = (*point)["timestamp"].as<double>();
            ++point;
            last_time = ros::Time::now();
          }
          if (as_.isPreemptRequested() || !ros::ok())
          {
            stopPlayback();
            break;
          }
        }
        if (point == points.end())
          duration.sleep();
        result_.succeed = true;
        as_.setSucceeded(result_);
      }
      catch (std::exception &e)
      {
        stopPlayback();
        ROS_WARN_STREAM("YAML parsing error: " << e.what());
      }
    }
  }

  void EngineerTrajectoryTeaching::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    joint_state_.clear();
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      joint_state_.insert(std::map<std::string, double>::value_type(msg->name[i], msg->position[i]));
    }
  }

  void EngineerTrajectoryTeaching::controllerStateCallback(const control_msgs::JointTrajectoryControllerStateConstPtr &msg)
  {
    controller_state_ = *msg;
  }


  void EngineerTrajectoryTeaching::record()
  {
    time_from_start_ = ros::Time::now() - start_record_time_;
    if ((time_from_start_ - last_time_from_start_).toSec() < record_interval_threshold_ )
      return;
    TrajectoryPoint point;
    point.timestamp = time_from_start_.toSec();
    for (const auto& record_joint_name : recorded_trajectory_.record_joint_names)
    {
      try
      {
        point.joint_position.push_back(joint_state_.at(record_joint_name));
      }
      catch (std::exception &e)
      {
        ROS_ERROR_STREAM("The joint named: " << record_joint_name << " was not found!");
        return;
      }
    }
    recorded_trajectory_.trajectory_points.push_back(point);
    feedback_.mode = "RECORD";
    feedback_.state = rm_msgs::EngineerTrajectoryTeachingFeedback::START;
    feedback_.text = "Recording...";
    feedback_.time_from_start  = time_from_start_.toSec();
    as_.publishFeedback(feedback_);
    last_time_from_start_ = time_from_start_;
  }

  void EngineerTrajectoryTeaching::startRecord()
  {
    recorded_trajectory_.trajectory_points.clear();
    start_record_time_ = ros::Time::now();
    last_time_from_start_ = ros::Duration(-500.0);
    ROS_INFO_STREAM("Start recording ...");
  }

  void EngineerTrajectoryTeaching::stopRecord(const std::string& file_path)
  {
    recordToYAML(file_path);
    ROS_INFO_STREAM("Saved trajectory to: " << file_path);
  }

  void EngineerTrajectoryTeaching::recordToYAML(const std::string& file_path)
  {
    try
    {
      std::ofstream f_out(file_path);
      YAML::Emitter emitter(f_out);
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << recorded_trajectory_.record_joint_names;
      emitter << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
      for (const auto& point : recorded_trajectory_.trajectory_points)
      {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "timestamp" << YAML::Value << point.timestamp;
        emitter << YAML::Key << "positions" << YAML::Value << YAML::Flow << point.joint_position;
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndSeq;
      emitter << YAML::EndMap;
      f_out.close();
      ROS_INFO("Save successfully");
    }
    catch (std::exception &e)
    {
      ROS_ERROR_STREAM("YAML parsing error: " << e.what());
    }
  }

  void EngineerTrajectoryTeaching::playback(const YAML::Node &point, ros::Duration& duration)
  {
    if (point["positions"].IsDefined())
    {
      trajectory_cmd_.points.clear();
      trajectory_cmd_.header.stamp = ros::Time(0);
      trajectory_msgs::JointTrajectoryPoint trajectory_point;
      const auto positions = point["positions"].as<std::vector<double>>();
      trajectory_point.positions = positions;
      if (point["velocities"].IsDefined())
      {
        const auto velocities = point["velocities"].as<std::vector<double>>();
        trajectory_point.velocities = velocities;
      }
      if (point["accelerations"].IsDefined())
      {
        const auto accelerations = point["accelerations"].as<std::vector<double>>();
        trajectory_point.accelerations = accelerations;
      }
      trajectory_point.time_from_start =  duration;
      trajectory_cmd_.points.push_back(trajectory_point);
      trajectory_cmd_pub_.publish(trajectory_cmd_);
    }
    if (point["small_joint1"].IsDefined())
    {
      std_msgs::Float64 small_joint1_cmd;
      small_joint1_cmd.data = point["small_joint1"].as<double>();
      small_joint1_cmd_pub_.publish(small_joint1_cmd);
    }
    if (point["small_joint2"].IsDefined())
    {
      std_msgs::Float64 small_joint2_cmd;
      small_joint2_cmd.data = point["small_joint2"].as<double>();
      small_joint2_cmd_pub_.publish(small_joint2_cmd);
    }
    if (point["small_joint3"].IsDefined())
    {
      std_msgs::Float64 small_joint3_cmd;
      small_joint3_cmd.data = point["small_joint3"].as<double>();
      small_joint3_cmd_pub_.publish(small_joint3_cmd);
    }
    if (point["main_gripper"].IsDefined() || point["small_gripper"].IsDefined() || point["transfer_gripper"].IsDefined())
    {
      rm_msgs::GpioData gpio_cmd;
      if (point["main_gripper"].IsDefined())
      {
        gpio_cmd.gpio_name.emplace_back("main_gripper");
        gpio_cmd.gpio_state.emplace_back(point["main_gripper"].as<bool>());
      }
      if (point["small_gripper"].IsDefined())
      {
        gpio_cmd.gpio_name.emplace_back("small_gripper");
        gpio_cmd.gpio_state.emplace_back(point["small_gripper"].as<bool>());
      }
      if (point["transfer_gripper"].IsDefined())
      {
        gpio_cmd.gpio_name.emplace_back("transfer_gripper");
        gpio_cmd.gpio_state.emplace_back(point["transfer_gripper"].as<bool>());
      }
      gpio_cmd_pub_.publish(gpio_cmd);
    }
    if (point["gimbal"].IsDefined())
    {
      rm_msgs::GimbalCmd gimbal_cmd;
      gimbal_cmd.mode = rm_msgs::GimbalCmd::DIRECT;
      gimbal_cmd.target_pos.header.frame_id = "base_link";
      gimbal_cmd.target_pos.point.x = point["gimbal"][0].as<double>();
      gimbal_cmd.target_pos.point.y = point["gimbal"][1].as<double>();
      gimbal_cmd.target_pos.point.z = point["gimbal"][2].as<double>();
      gimbal_cmd_pub_.publish(gimbal_cmd);
    }
    if (point["middle_pitch"].IsDefined())
    {
      std_msgs::Float64 middle_pitch_cmd;
      middle_pitch_cmd.data = point["middle_pitch"].as<double>();
      middle_pitch_cmd_pub_.publish(middle_pitch_cmd);
    }
    feedback_.mode = "PLAYBACK";
    feedback_.state = rm_msgs::EngineerTrajectoryTeachingFeedback::START;
    feedback_.text = "Playback...";
    feedback_.time_from_start  = point["timestamp"].as<double>();
    as_.publishFeedback(feedback_);
  }

  void EngineerTrajectoryTeaching::stopPlayback()
  {
    trajectory_cmd_.points.clear();
    trajectory_cmd_.header.stamp = ros::Time(0);
    trajectory_cmd_.joint_names = controller_state_.joint_names;
    trajectory_msgs::JointTrajectoryPoint current_point;
    current_point.positions = controller_state_.actual.positions;
    current_point.time_from_start =  ros::Duration(0.1);
    trajectory_cmd_.points.push_back(current_point);
    trajectory_cmd_pub_.publish(trajectory_cmd_);
    ROS_INFO_STREAM("Stop playback");
  }

}