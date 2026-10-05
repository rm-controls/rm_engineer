//
// Created by ch on 24-12-6.
//

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <class_loader/class_loader.hpp>
#include "auto_exchange_planner/auto_exchange_context.h"

namespace auto_exchange_planner
{
class AutoExchangeManager : public planning_interface::PlannerManager
{
public:
  AutoExchangeManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override
  {
    for (const std::string& gpName : model->getJointModelGroupNames())
    {
      planning_contexts_[gpName] =
          AutoExchangeContextPtr(new AutoExchangeContext("auto_exchange_context", ns, gpName, model));
    }
    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "AutoExchange";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("auto_exchange");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // retrieve and configure existing context
    const AutoExchangeContextPtr& context = planning_contexts_.at(req.group_name);
    ROS_INFO_STREAM_NAMED("auto_exchange_manager", "===>>> context is made ");

    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
  }

protected:
  std::map<std::string, AutoExchangeContextPtr> planning_contexts_;
};

}  // namespace auto_exchange_planner

// register the AutoExchangeManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(auto_exchange_planner::AutoExchangeManager, planning_interface::PlannerManager);
