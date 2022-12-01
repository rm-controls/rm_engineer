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
// Created by qiayuan on 4/3/21.
//

#ifndef ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#define ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#pragma once

#include "engineer_middleware/step.h"

// STL
#include <deque>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <rm_msgs/EngineerAction.h>

namespace engineer_middleware
{
class StepQueue
{
public:
  StepQueue(const XmlRpc::XmlRpcValue& steps, const XmlRpc::XmlRpcValue& scenes, tf2_ros::Buffer& tf,
            moveit::planning_interface::MoveGroupInterface& arm_group, ChassisInterface& chassis_interface,
            ros::Publisher& hand_pub, ros::Publisher& card_pub, ros::Publisher& gimbal_pub, ros::Publisher& gpio_pub)
    : chassis_interface_(chassis_interface)
  {
    ROS_ASSERT(steps.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < steps.size(); ++i)
    {
      queue_.emplace_back(steps[i], scenes, tf, arm_group, chassis_interface, hand_pub, card_pub, gimbal_pub, gpio_pub);
    }
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = scenes.begin(); it != scenes.end(); ++it)
      for (int i = 0; i < it->second.size(); i++)
        object_ids_.push_back(it->second[i]["id"]);
  }
  bool run(actionlib::SimpleActionServer<rm_msgs::EngineerAction>& as, geometry_msgs::TwistStamped target_twist)
  {
    if (queue_.empty())
    {
      ROS_WARN("Step queue is empty");
      return false;
    }
    chassis_interface_.setCurrentAsGoal();
    rm_msgs::EngineerFeedback feedback;
    rm_msgs::EngineerResult result;
    feedback.total_steps = queue_.size();
    for (size_t i = 0; i < queue_.size(); ++i)
    {
      ros::Time start = ros::Time::now();
      if (!queue_[i].move(target_twist))
        return false;
      ROS_INFO("Start step: %s", queue_[i].getName().c_str());
      while (!queue_[i].isFinish())
      {
        if (!queue_[i].checkTimeout(ros::Time::now() - start))
        {
          queue_[i].stop();
          return false;
        }
        if (as.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("Step %s Preempted", queue_[i].getName().c_str());
          queue_[i].stop();
          as.setPreempted();
          return false;
        }
        feedback.finished_step = i;
        feedback.current_step = queue_[i].getName();
        as.publishFeedback(feedback);
        ros::Duration(0.01).sleep();
      }
      feedback.finished_step = queue_.size();
      as.publishFeedback(feedback);
      ROS_INFO("Finish step: %s", queue_[i].getName().c_str());
    }
    result.finish = true;
    deleteScene();
    as.setSucceeded(result);
    return true;
  }
  void deleteScene()
  {
    queue_.begin()->deleteScene(object_ids_);
  }
  const std::deque<Step>& getQueue() const
  {
    return queue_;
  }
  std::deque<Step>::size_type size() const
  {
    return queue_.size();
  }

private:
  std::deque<Step> queue_;
  std::vector<std::string> object_ids_;
  ChassisInterface& chassis_interface_;
};
}  // namespace engineer_middleware

#endif  // ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
