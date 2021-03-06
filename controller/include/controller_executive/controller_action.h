/**
 *
 * @file controller_action.h
 * @brief Contains the action server for the controller
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef EXECUTIVE_CONTROLLER_ACTION_H
#define EXECUTIVE_CONTROLLER_ACTION_H

#include "mw_core/abstract_classes/abstract_action_base.h"

#include <controller_executive/base_controller_execution.h>

#include <mw_core/robot_information.h>
#include <mw_msgs/ExePathAction.h>

#include <actionlib/server/action_server.h>

namespace controller {

class ControllerAction : public mw_core::AbstractActionBase<mw_msgs::ExePathAction, controller::AbstractControllerExecution> {

 public:

  typedef boost::shared_ptr<ControllerAction> Ptr;

  ControllerAction(const std::string &name, const mw_core::RobotInformation &robot_info);

  /**
   * @brief Start controller action.
   * Override abstract action version to allow updating current plan without stopping execution.
   * @param goal_handle Reference to the goal handle received on action execution callback.
   * @param execution_ptr Pointer to the execution descriptor.
   */
  void start(
      GoalHandle &goal_handle,
      typename controller::AbstractControllerExecution::Ptr execution_ptr) override;

  void run(GoalHandle &goal_handle, controller::AbstractControllerExecution &execution);

 protected:
  void publishExePathFeedback(GoalHandle &goal_handle,
                              uint32_t outcome,
                              const std::string &message,
                              const geometry_msgs::TwistStamped &current_twist);

  /**
   * @brief Utility method to fill the ExePath action result in a single line
   * @param outcome ExePath action outcome
   * @param message ExePath action message
   * @param result The action result to fill
   */
  void fillExePathResult(uint32_t outcome,
                         const std::string &message,
                         mw_msgs::ExePathResult &result);

  boost::mutex goal_mtx_; ///< lock goal handle for updating it while running
  geometry_msgs::PoseStamped robot_pose_; ///< Current robot pose
  geometry_msgs::PoseStamped goal_pose_;  ///< Current goal pose

};
}

#endif //EXECUTIVE_CONTROLLER_ACTION_H
