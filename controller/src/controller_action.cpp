/**
 *
 * @file controller_action.cpp
 * @brief Contains the action server for the controller
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/controller_action.h"

namespace controller {

ControllerAction::ControllerAction(
    const std::string &action_name,
    const mw_core::RobotInformation &robot_info)
    : AbstractActionBase(action_name, robot_info, boost::bind(&controller::ControllerAction::run, this, _1, _2)) {
}

void ControllerAction::start(
    GoalHandle &goal_handle,
    typename controller::AbstractControllerExecution::Ptr execution_ptr) {
  if (goal_handle.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING) {
    goal_handle.setCanceled();
    return;
  }

  uint8_t slot = goal_handle.getGoal()->concurrency_slot;

  bool update_plan = false;
  slot_map_mtx_.lock();
  auto slot_it = concurrency_slots_.find(slot);
  if (slot_it != concurrency_slots_.end()) {
    boost::lock_guard<boost::mutex> goal_guard(goal_mtx_);
    if (slot_it->second.execution->getName() == goal_handle.getGoal()->controller ||
        goal_handle.getGoal()->controller.empty()) {
      update_plan = true;
      // Goal requests to run the same controller on the same concurrency slot:
      // we update the goal handle and pass the new plan to the execution without stopping it
      execution_ptr = slot_it->second.execution;
      execution_ptr->setNewPlan(goal_handle.getGoal()->path.poses);
      // Update also goal pose, so the feedback remains consistent
      goal_pose_ = goal_handle.getGoal()->path.poses.back();
      mw_msgs::ExePathResult result;
      fillExePathResult(mw_msgs::ExePathResult::CANCELED, "Goal preempted by a new plan", result);
      concurrency_slots_[slot].goal_handle.setCanceled(result, result.message);
      concurrency_slots_[slot].goal_handle = goal_handle;
      concurrency_slots_[slot].goal_handle.setAccepted();
    }
  }
  slot_map_mtx_.unlock();
  if (!update_plan) {
    // Otherwise run parent version of this method
    AbstractActionBase::start(goal_handle, execution_ptr);
  }
}

void ControllerAction::run(GoalHandle &goal_handle, controller::AbstractControllerExecution &execution) {
  goal_mtx_.lock();
  // Note that we always use the goal handle stored on the concurrency slots map, as it can change when replanning
  uint8_t slot = goal_handle.getGoal()->concurrency_slot;
  goal_mtx_.unlock();

  ROS_DEBUG_STREAM_NAMED(name_, "Start action " << name_);

  // ensure we don't provide values from previous execution on case of error before filling both poses
  goal_pose_ = geometry_msgs::PoseStamped();
  robot_pose_ = geometry_msgs::PoseStamped();

  ros::NodeHandle private_nh("~");

  double oscillation_timeout_tmp;
  private_nh.param("oscillation_timeout", oscillation_timeout_tmp, 0.0);
  ros::Duration oscillation_timeout(oscillation_timeout_tmp);

  double oscillation_distance;
  private_nh.param("oscillation_distance", oscillation_distance, 0.03);

  mw_msgs::ExePathResult result;
  mw_msgs::ExePathFeedback feedback;

  typename controller::AbstractControllerExecution::ControllerState state_moving_input;
  bool controller_active = true;

  goal_mtx_.lock();
  const mw_msgs::ExePathGoal &goal = *(goal_handle.getGoal().get());

  const std::vector<geometry_msgs::PoseStamped> &plan = goal.path.poses;
  if (plan.empty()) {
    fillExePathResult(mw_msgs::ExePathResult::INVALID_PATH, "Controller started with an empty plan!", result);
    goal_handle.setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
    controller_active = false;
  }

  goal_pose_ = plan.back();
  ROS_DEBUG_STREAM_NAMED(name_, "Called action \""
      << name_ << "\" with plan:" << std::endl
      << "frame: \"" << goal.path.header.frame_id << "\" " << std::endl
      << "stamp: " << goal.path.header.stamp << std::endl
      << "poses: " << goal.path.poses.size() << std::endl
      << "goal: (" << goal_pose_.pose.position.x << ", "
      << goal_pose_.pose.position.y << ", "
      << goal_pose_.pose.position.z << ")");

  goal_mtx_.unlock();

  geometry_msgs::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  while (controller_active && ros::ok()) {
    // goal_handle could change between the loop cycles due to adapting the plan
    // with a new goal received for the same concurrency slot
    if (!robot_info_.getRobotPose(robot_pose_)) {
      controller_active = false;
      fillExePathResult(mw_msgs::ExePathResult::TF_ERROR, "Could not get the robot pose!", result);
      goal_mtx_.lock();
      goal_handle.setAborted(result, result.message);
      goal_mtx_.unlock();
      ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
      break;
    }

    if (first_cycle) {
      // init oscillation pose
      oscillation_pose = robot_pose_;
    }

    goal_mtx_.lock();
    state_moving_input = execution.getState();

    switch (state_moving_input) {
      case controller::AbstractControllerExecution::INITIALIZED:execution.setNewPlan(plan);
        execution.start();
        break;

      case controller::AbstractControllerExecution::STOPPED:ROS_WARN_STREAM_NAMED(name_,
                                                                                  "The controller has been stopped!");
        controller_active = false;
        break;

      case controller::AbstractControllerExecution::CANCELED:ROS_INFO_STREAM("Action \"exe_path\" canceled");
        fillExePathResult(mw_msgs::ExePathResult::CANCELED, "Controller canceled", result);
        goal_handle.setCanceled(result, result.message);
        controller_active = false;
        break;

      case controller::AbstractControllerExecution::STARTED:ROS_DEBUG_STREAM_NAMED(name_,
                                                                                   "The moving has been started!");
        break;

        // in progress
      case controller::AbstractControllerExecution::PLANNING:
        if (execution.isPatienceExceeded()) {
          ROS_INFO_STREAM_NAMED(name_, "The controller patience has been exceeded! Stopping controller...");
          // TODO planner is stuck, but we don't have currently any way to cancel it!
          // We will try to stop the thread, but does nothing with DWA, TR or TEB controllers
          // Note that this is not the same situation as in case AbstractControllerExecution::PAT_EXCEEDED,
          // as there is the controller itself reporting that it cannot find a valid command after trying
          // for more than patience seconds. But after stopping controller execution, it should ideally
          // report PAT_EXCEEDED as his state on next iteration.
          execution.stop();
        }
        break;

      case controller::AbstractControllerExecution::MAX_RETRIES:ROS_WARN_STREAM_NAMED(name_,
                                                                                      "The controller has been aborted after it exceeded the maximum number of retries!");
        controller_active = false;
        fillExePathResult(execution.getOutcome(), execution.getMessage(), result);
        goal_handle.setAborted(result, result.message);
        break;

      case controller::AbstractControllerExecution::PAT_EXCEEDED:ROS_WARN_STREAM_NAMED(name_,
                                                                                       "The controller has been aborted after it exceeded the patience time");
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::PAT_EXCEEDED, execution.getMessage(), result);
        goal_handle.setAborted(result, result.message);
        break;

      case controller::AbstractControllerExecution::NO_PLAN:ROS_WARN_STREAM_NAMED(name_,
                                                                                  "The controller has been started without a plan!");
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::INVALID_PATH, "Controller started without a path", result);
        goal_handle.setAborted(result, result.message);
        break;

      case controller::AbstractControllerExecution::EMPTY_PLAN:ROS_WARN_STREAM_NAMED(name_,
                                                                                     "The controller has received an empty plan");
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::INVALID_PATH, "Controller started with an empty plan", result);
        goal_handle.setAborted(result, result.message);
        break;

      case controller::AbstractControllerExecution::INVALID_PLAN:ROS_WARN_STREAM_NAMED(name_,
                                                                                       "The controller has received an invalid plan");
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::INVALID_PATH, "Controller started with an invalid plan", result);
        goal_handle.setAborted(result, result.message);
        break;

      case controller::AbstractControllerExecution::NO_LOCAL_CMD:ROS_WARN_STREAM_THROTTLE_NAMED(3,
                                                                                                name_,
                                                                                                "No velocity command received from controller! "
                                                                                                    << execution.getMessage());
        publishExePathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
        break;

      case controller::AbstractControllerExecution::GOT_LOCAL_CMD:
        if (!oscillation_timeout.isZero()) {
          // check if oscillating
          if (mw_core::distance(robot_pose_, oscillation_pose) >= oscillation_distance) {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose_;
          } else if (last_oscillation_reset + oscillation_timeout < ros::Time::now()) {
            ROS_WARN_STREAM_NAMED(name_, "The controller is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
            execution.stop();
            controller_active = false;
            fillExePathResult(mw_msgs::ExePathResult::OSCILLATION, "Oscillation detected!", result);
            goal_handle.setAborted(result, result.message);
            break;
          }
        }
        publishExePathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
        break;

      case controller::AbstractControllerExecution::ARRIVED_GOAL:ROS_DEBUG_STREAM_NAMED(name_,
                                                                                        "Controller succeeded; arrived at goal");
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::SUCCESS, "Controller succeeded; arrived at goal!", result);
        goal_handle.setSucceeded(result, result.message);
        break;

      case controller::AbstractControllerExecution::INTERNAL_ERROR:ROS_FATAL_STREAM_NAMED(name_,
                                                                                          "Internal error: Unknown error thrown by the plugin: "
                                                                                              << execution.getMessage());
        controller_active = false;
        fillExePathResult(mw_msgs::ExePathResult::INTERNAL_ERROR,
                          "Internal error: Unknown error thrown by the plugin!",
                          result);
        goal_handle.setAborted(result, result.message);
        break;

      default:std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex controller execution with the number: "
           << static_cast<int>(state_moving_input);
        fillExePathResult(mw_msgs::ExePathResult::INTERNAL_ERROR, ss.str(), result);
        ROS_FATAL_STREAM_NAMED(name_, result.message);
        goal_handle.setAborted(result, result.message);
        controller_active = false;
    }
    goal_mtx_.unlock();

    if (controller_active) {
      // try to sleep a bit
      // normally this thread should be woken up from the controller execution thread
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }

    first_cycle = false;
  }  // while (controller_active && ros::ok())

  if (!controller_active) {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  } else {
    // normal on continuous replanning
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

void ControllerAction::publishExePathFeedback(
    GoalHandle &goal_handle,
    uint32_t outcome, const std::string &message,
    const geometry_msgs::TwistStamped &current_twist) {

  // ROS_INFO("[Controller executive controller_action]:Feedback given");    
  mw_msgs::ExePathFeedback feedback;
  feedback.outcome = outcome;
  feedback.message = message;

  feedback.last_cmd_vel = current_twist;
  if (feedback.last_cmd_vel.header.stamp.isZero())
    feedback.last_cmd_vel.header.stamp = ros::Time::now();

  feedback.current_pose = robot_pose_;
  feedback.dist_to_goal = static_cast<float>(mw_core::distance(robot_pose_, goal_pose_));
  feedback.angle_to_goal = static_cast<float>(mw_core::angle(robot_pose_, goal_pose_));
  goal_handle.publishFeedback(feedback);
}

void ControllerAction::fillExePathResult(
    uint32_t outcome, const std::string &message,
    mw_msgs::ExePathResult &result) {
  result.outcome = outcome;
  result.message = message;
  result.final_pose = robot_pose_;
  result.dist_to_goal = static_cast<float>(mw_core::distance(robot_pose_, goal_pose_));
  result.angle_to_goal = static_cast<float>(mw_core::angle(robot_pose_, goal_pose_));
}

}