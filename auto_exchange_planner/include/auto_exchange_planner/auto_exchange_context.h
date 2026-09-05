//
// Created by ch on 24-12-6.
//

#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include "auto_exchange_planner/auto_exchange_planner.h"

namespace auto_exchange_planner
{
  MOVEIT_CLASS_FORWARD(AutoExchangeContext);

  class AutoExchangeContext : public planning_interface::PlanningContext
  {
  public:
    AutoExchangeContext(const std::string& name, const std::string& ns, const std::string& group,
                        const moveit::core::RobotModelConstPtr& model);
    ~AutoExchangeContext() override
    {
    }

    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

    bool terminate() override;
    void clear() override;

  private:
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    AutoExchangePlannerPtr auto_exchange_planner_;
  };

}  // namespace auto_exchange_planner
