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

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware
{
Middleware::Middleware(ros::NodeHandle& nh)
  : nh_(nh)
  , as_(
        nh_, "move_steps", [this](auto&& PH1) { executeCB(std::forward<decltype(PH1)>(PH1)); }, false)
  , arm_group_(moveit::planning_interface::MoveGroupInterface("engineer_arm"))
  , chassis_interface_(nh, tf_)
  , hand_pub_(nh.advertise<std_msgs::Float64>("/controllers/hand_controller/command", 10))
  , end_effector_pub_(nh.advertise<std_msgs::Float64>("/controllers/joint7_controller/command", 10))
  , gimbal_pub_(nh.advertise<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10))
  , gpio_pub_(nh.advertise<rm_msgs::GpioData>("/controllers/gpio_controller/command", 10))
  , reversal_pub_(nh.advertise<rm_msgs::MultiDofCmd>("/controllers/multi_dof_controller/command", 10))
  , planning_result_pub_(nh.advertise<std_msgs::Int32>("/planning_result", 10))
  , stone_num_pub_(nh.advertise<std_msgs::String>("/stone_num", 10))
  , point_cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>("/cloud", 100))
  , ore_rotate_pub_(nh.advertise<std_msgs::Float64>("/controllers/ore_bin_rotate_controller/command", 10))
  , ore_lift_pub_(nh.advertise<std_msgs::Float64>("/controllers/ore_bin_lifter_controller/command", 10))
  , gimbal_lift_pub_(nh.advertise<std_msgs::Float64>("/controllers/gimbal_lifter_controller/command", 10))
  , extend_arm_f_pub_(nh.advertise<std_msgs::Float64>("/controllers/extend_arm_front_controller/command", 10))
  , extend_arm_b_pub_(nh.advertise<std_msgs::Float64>("/controllers/extend_arm_back_controller/command", 10))
  , silver_lifter_pub_(nh.advertise<std_msgs::Float64>("/controllers/silver_lifter_controller/command", 10))
  , silver_pusher_pub_(nh.advertise<std_msgs::Float64>("/controllers/silver_pusher_controller/command", 10))
  , silver_rotator_pub_(nh.advertise<std_msgs::Float64>("/controllers/silver_rotator_controller/command", 10))
  , gold_pusher_pub_(nh.advertise<std_msgs::Float64>("/controllers/gold_pusher_controller/command", 10))
  , gold_lifter_pub_(nh.advertise<std_msgs::Float64>("/controllers/gold_lifter_controller/command", 10))
  , middle_pitch_pub_(nh.advertise<std_msgs::Float64>("/controllers/middle_pitch_controller/command", 10))
  , tf_listener_(tf_)
  , is_middleware_control_(false)
{
  if (nh.hasParam("steps_list") && nh.hasParam("scenes_list"))
  {
    XmlRpc::XmlRpcValue steps_list;
    XmlRpc::XmlRpcValue scenes_list;
    nh.getParam("steps_list", steps_list);
    nh.getParam("scenes_list", scenes_list);
    ROS_ASSERT(steps_list.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct);
    ROS_ASSERT(scenes_list.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = steps_list.begin(); it != steps_list.end(); ++it)
    {
      step_queues_.insert(std::make_pair(
          it->first, StepQueue(it->second, scenes_list, tf_, arm_group_, chassis_interface_, hand_pub_,
                               end_effector_pub_, gimbal_pub_, gpio_pub_, reversal_pub_, stone_num_pub_,
                               planning_result_pub_, point_cloud_pub_, ore_rotate_pub_, ore_lift_pub_, gimbal_lift_pub_,
                               extend_arm_f_pub_, extend_arm_b_pub_, silver_lifter_pub_, silver_pusher_pub_,
                               silver_rotator_pub_, gold_pusher_pub_, gold_lifter_pub_, middle_pitch_pub_)));
    }
  }
  else
    ROS_ERROR("no steps list define in yaml");
  as_.start();
}
geometry_msgs::TransformStamped engineer_middleware::JointMotion::arm2base;

}  // namespace engineer_middleware
