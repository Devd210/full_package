/**
 *
 * @file planner_action.h
 * @brief Contains the action server for the planner
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#pragma once

#include <mw_core/abstract_classes/abstract_action_base.h>
#include <global_planner/base_planner_execution.h>
#include <actionlib/server/action_server.h>
#include <mw_msgs/GetPathAction.h>

namespace global_planner {
/**
 * @class PlannerAction
 * @brief Contains the defination of the planner action which is based on the abstract action base
 */
class PlannerAction : public mw_core::AbstractActionBase<mw_msgs::GetPathAction,
                                                         global_planner::AbstractPlannerExecution> {
 public:
  typedef boost::shared_ptr<PlannerAction> Ptr;
  /**
   * @brief Constructor to init the action
   */
  PlannerAction(const std::string &name,
                const mw_core::RobotInformation &robot_info);

  //The run method that is run when a new goal is received
  void run(GoalHandle &goal_handle, global_planner::AbstractPlannerExecution &execution);

 protected:
  //Transform a given plan to a global plan
  bool transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped> &plan,
                                  std::vector<geometry_msgs::PoseStamped> &global_plan);

 private:
  ros::Publisher current_goal_pub_;
  unsigned int path_seq_count_;
};
} // namespace global_planner
