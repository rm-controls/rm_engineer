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
// Created by astro on 2021/4/25.
//

#pragma once

#include "engineer_middleware/step_queue.h"
#include "engineer_middleware/planning_scene.h"
#include <tf/transform_broadcaster.h>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <rm_msgs/EngineerAction.h>
#include <rm_msgs/GpioData.h>

namespace engineer_middleware
{
class Middleware
{
public:
  explicit Middleware(ros::NodeHandle& nh);
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr& goal)
  {
    std::string name;
    name = goal->step_queue_name;
    is_middleware_control_ = true;
    ROS_INFO("Start step queue id %s", name.c_str());
    auto step_queue = step_queues_.find(name);
    if (step_queue != step_queues_.end())
      step_queue->second.run(as_, target_pos_);
    ROS_INFO("Finish step queue id %s", name.c_str());
    is_middleware_control_ = false;
  }
  void targetPosDataCallback(const geometry_msgs::PoseStamped::ConstPtr& data)
  {
    target_pos_ = *data;
  }
  void run(ros::Duration period)
  {
    if (is_middleware_control_)
      chassis_interface_.run(period);
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z));
    transform.setRotation(tf::Quaternion(target_pos_.pose.orientation.x, target_pos_.pose.orientation.y,
                                         target_pos_.pose.orientation.z, target_pos_.pose.orientation.w));
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "target_link"));
  }

private:
  tf::TransformBroadcaster br_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> as_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  ChassisInterface chassis_interface_;
  ros::Publisher hand_pub_, card_pub_, gimbal_pub_, gpio_pub_;
  ros::Subscriber target_sub_;
  geometry_msgs::PoseStamped target_pos_;
  std::unordered_map<std::string, StepQueue> step_queues_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tf_listener_;
  bool is_middleware_control_;
};

}  // namespace engineer_middleware
