//
// Created by ch on 25-7-6.
//

#pragma once

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <rm_msgs/EngineerTrajectoryTeachingAction.h>
#include <rm_msgs/GpioData.h>
#include <rm_msgs/GimbalCmd.h>

namespace  engineer_trajectory_teaching
{
class EngineerTrajectoryTeaching
{
public:
    explicit EngineerTrajectoryTeaching(ros::NodeHandle& nh);
    void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerTrajectoryTeachingAction>::GoalConstPtr& goal);

private:
    void startRecord();
    void record();
    void stopRecord(const std::string& file_path);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data);
    void controllerStateCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
    void recordToYAML(const std::string& file_path);
    void playback(const YAML::Node& point, ros::Duration& duration);
    void stopPlayback();

    struct TrajectoryPoint
    {
        double timestamp;
        std::vector<double>  joint_position;
    };
    struct Trajectory
    {
        std::vector<std::string> record_joint_names;
        std::vector<TrajectoryPoint> trajectory_points;
    } recorded_trajectory_;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rm_msgs::EngineerTrajectoryTeachingAction> as_;
    rm_msgs::EngineerTrajectoryTeachingFeedback feedback_;
    rm_msgs::EngineerTrajectoryTeachingResult result_;
    control_msgs::JointTrajectoryControllerState controller_state_;
    trajectory_msgs::JointTrajectory trajectory_cmd_;
    ros::Subscriber joint_state_sub_, controller_state_sub_;
    ros::Publisher trajectory_cmd_pub_, small_joint1_cmd_pub_, small_joint2_cmd_pub_,
                    small_joint3_cmd_pub_, gpio_cmd_pub_, gimbal_cmd_pub_, middle_pitch_cmd_pub_;
    std::map<std::string, double> joint_state_;
    ros::Time start_record_time_;
    ros::Duration last_time_from_start_, time_from_start_;
    std::string file_path_;
    double record_interval_threshold_ = 0.1, sync_time_ = 0.1,speed_ = 1.0;
};
}
