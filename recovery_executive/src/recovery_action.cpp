/**
 *
 * @file recovery_action.cpp
 * @brief recovery action class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/recovery_action.h"

namespace recovery {

RecoveryAction::RecoveryAction(const std::string &name, const mw_core::RobotInformation &robot_info)
  : AbstractActionBase(name, robot_info, boost::bind(&recovery::RecoveryAction::run, this, _1, _2)){}

void RecoveryAction::run(GoalHandle &goal_handle, AbstractRecoveryExecution &execution)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  const mw_msgs::RecoveryGoal &goal = *(goal_handle.getGoal().get());
  mw_msgs::RecoveryResult result;
  bool recovery_active = true;

  typename AbstractRecoveryExecution::RecoveryState state_recovery_input;

  while (recovery_active && ros::ok())
  {
    state_recovery_input = execution.getState();
    switch (state_recovery_input)
    {
      case AbstractRecoveryExecution::INITIALIZED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior \"" << goal.behavior << "\" initialized.");
        execution.start();
        break;

      case AbstractRecoveryExecution::STOPPED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior stopped rigorously");
        result.outcome = mw_msgs::RecoveryResult::STOPPED;
        result.message = "Recovery has been stopped!";
        goal_handle.setAborted(result, result.message);
        recovery_active = false;
        break;

      case AbstractRecoveryExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior \"" << goal.behavior << "\" was started");
        break;

      case AbstractRecoveryExecution::RECOVERING:

        if (execution.isPatienceExceeded())
        {
          ROS_INFO_STREAM("Recovery behavior \"" << goal.behavior << "\" patience exceeded! Cancel recovering...");
          if (!execution.cancel())
          {
            ROS_WARN_STREAM("Cancel recovering \"" << goal.behavior << "\" failed or not supported; maybe wait until it is finished!");
          }
        }

        ROS_DEBUG_STREAM_THROTTLE_NAMED(3, name_, "Recovering with: " << goal.behavior);
        break;

      case AbstractRecoveryExecution::CANCELED:
        // Recovery behavior supports cancel and it worked
        recovery_active = false; // stopping the action
        result.outcome = mw_msgs::RecoveryResult::CANCELED;
        result.message = "Recovery behaviour \"" + goal.behavior + "\" canceled!";
        goal_handle.setCanceled(result, result.message);
        ROS_DEBUG_STREAM_NAMED(name_, result.message);
        break;

      case AbstractRecoveryExecution::RECOVERY_DONE:
        recovery_active = false; // stopping the action
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        if (result.message.empty())
        {
          if (result.outcome < 10)
            result.message = "Recovery \"" + goal.behavior + "\" done";
          else
            result.message = "Recovery \"" + goal.behavior + "\" FAILED";
        }

        ROS_DEBUG_STREAM_NAMED(name_, result.message);
        goal_handle.setSucceeded(result, result.message);
        break;

      case AbstractRecoveryExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from recovery
        recovery_active = false;
        result.outcome = mw_msgs::RecoveryResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.setAborted(result, result.message);
        break;

      default:
        result.outcome = mw_msgs::RecoveryResult::INTERNAL_ERROR;
        std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex recovery execution with the number: "
           << static_cast<int>(state_recovery_input);
        result.message = ss.str();
        ROS_FATAL_STREAM_NAMED(name_, result.message);
        goal_handle.setAborted(result, result.message);
        recovery_active = false;
    }

    if (recovery_active)
    {
      // try to sleep a bit
      // normally the thread should be woken up from the recovery unit
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }
  }  // while (recovery_active && ros::ok())

  if (!recovery_active)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

} /* namespace recovery */
