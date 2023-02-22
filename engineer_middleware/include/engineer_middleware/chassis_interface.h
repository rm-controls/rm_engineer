/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 5/30/21.
//

#pragma once

#include <rm_common/ori_tool.h>

#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace engineer_middleware
{
class ChassisInterface
{
public:
  explicit ChassisInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf) : tf_(tf)
  {
    ros::NodeHandle nh_base_motion = ros::NodeHandle(nh, "chassis");
    ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_base_motion, "x");
    ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_base_motion, "y");
    ros::NodeHandle nh_pid_w = ros::NodeHandle(nh_base_motion, "yaw");
    pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
    pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
    pid_yaw_.init(ros::NodeHandle(nh_pid_w, "pid"));
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    XmlRpc::XmlRpcValue chassis_common;
    nh.getParam("chassis_common", chassis_common);
    yaw_start_threshold_ = xmlRpcGetDouble(chassis_common, "yaw_start_threshold", 0.0);
  }

  bool setGoal(const geometry_msgs::PoseStamped& pose)
  {
    goal_ = pose;
    try
    {
      tf2::doTransform(goal_, goal_, tf_.lookupTransform("map", goal_.header.frame_id, ros::Time(0)));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    error_pos_ = 1e10;
    error_yaw_ = 1e10;
    return true;
  };

  void setCurrentAsGoal()
  {
    geometry_msgs::PoseStamped current{};
    current.header.frame_id = "base_link";
    current.pose.orientation.w = 1.;
    setGoal(current);
  }

  double getErrorPos() const
  {
    return error_pos_;
  }
  double getErrorYaw() const
  {
    return error_yaw_;
  }

  void stop()
  {
    geometry_msgs::Twist cmd_vel{};
    cmd_vel.linear.x = 0.;
    cmd_vel.linear.y = 0.;
    cmd_vel.angular.z = 0.;
    vel_pub_.publish(cmd_vel);
  }

  void run(ros::Duration period)
  {
    geometry_msgs::TransformStamped current;
    try
    {
      current = tf_.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    // Transform xy error under map frame to velocity under chassis
    geometry_msgs::Vector3 error;
    error.x = goal_.pose.position.x - current.transform.translation.x;
    error.y = goal_.pose.position.y - current.transform.translation.y;
    try
    {
      tf2::doTransform(error, error, tf_.lookupTransform("base_link", "map", ros::Time(0)));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    double roll, pitch, yaw_current, yaw_goal;
    quatToRPY(current.transform.rotation, roll, pitch, yaw_current);
    quatToRPY(goal_.pose.orientation, roll, pitch, yaw_goal);
    error_yaw_ = angles::shortest_angular_distance(yaw_current, yaw_goal);
    geometry_msgs::Twist cmd_vel_{};
    cmd_vel_.linear.x = pid_x_.computeCommand(error.x, period);
    cmd_vel_.linear.y = pid_y_.computeCommand(error.y, period);
    cmd_vel_.angular.z = (pid_yaw_.computeCommand(error_yaw_, period) >= yaw_start_threshold_ ?
                              pid_yaw_.computeCommand(error_yaw_, period) :
                              0);
    ;
    vel_pub_.publish(cmd_vel_);
    error_pos_ = std::abs(error.x) + std::abs(error.y);
    error_yaw_ = std::abs(error_yaw_);
  }

private:
  tf2_ros::Buffer& tf_;
  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;
  geometry_msgs::PoseStamped goal_{};
  ros::Publisher vel_pub_;
  double error_pos_{}, error_yaw_{};
  double yaw_start_threshold_{};
};
}  // namespace engineer_middleware
