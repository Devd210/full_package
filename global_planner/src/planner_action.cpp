/**
 *
 * @file planner_action.cpp
 * @brief Contains the implementation of the functions defined in the header file
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#include <global_planner/planner_action.h>

namespace global_planner {

PlannerAction::PlannerAction(const std::string &name,
                             const mw_core::RobotInformation &robot_info)
    : AbstractActionBase(name, robot_info, boost::bind(&PlannerAction::run, this, _1, _2)), path_seq_count_(0) {
  ros::NodeHandle private_nh("~");
  // informative topics: current navigation goal
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
}

void PlannerAction::run(GoalHandle &goal_handle, global_planner::AbstractPlannerExecution &execution) {
  execution.preRun();
  const mw_msgs::GetPathGoal &goal = *(goal_handle.getGoal().get());

  mw_msgs::GetPathResult result;
  geometry_msgs::PoseStamped start_pose;

  result.path.header.seq = path_seq_count_++;
  result.path.header.frame_id = robot_info_.getGlobalFrame();

  double tolerance = goal.tolerance;
  bool use_current_pose = goal.use_current_pose;
  current_goal_pub_.publish(goal.waypoints.back());

  bool planner_active = true;
  std::vector<geometry_msgs::PoseStamped> waypoints;

  if (!use_current_pose) {
    start_pose = goal.waypoints.front();
    const geometry_msgs::Point &p = start_pose.pose.position;
    waypoints = goal.waypoints;
    ROS_DEBUG_STREAM("[Planner Action]: Use the given start pose (" << p.x << ", " << p.y << ", " << p.z << ").");
  } else {
    // get the current robot pose
    if (!robot_info_.getRobotPose(start_pose)) {
      result.outcome = mw_msgs::GetPathResult::TF_ERROR;
      result.message = "Could not get the current robot pose!";
      goal_handle.setAborted(result, result.message);
      ROS_ERROR_STREAM("[Planner Action]: " << result.message << " Canceling the action call.");
      return;
    } else {
      waypoints.push_back(start_pose);
      waypoints.insert(waypoints.end(), goal.waypoints.begin(), goal.waypoints.end());
      const geometry_msgs::Point &p = start_pose.pose.position;
      ROS_DEBUG_STREAM(
          "[Planner Action]: Got the current robot pose at (" << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  global_planner::AbstractPlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::PoseStamped> plan, global_plan;

  while (planner_active && ros::ok()) {
    // get the current state of the planning thread
    state_planning_input = execution.getState();

    switch (state_planning_input) {
      case AbstractPlannerExecution::INITIALIZED:ROS_DEBUG_STREAM("[Planner Action]: "
                                                                              << "planner state: initialized");
        if (!execution.start(waypoints, tolerance)) {
          result.outcome = mw_msgs::GetPathResult::INTERNAL_ERROR;
          result.message = "Another thread is still planning!";
          goal_handle.setAborted(result, result.message);
          ROS_ERROR_STREAM("[Planner Action]: " << result.message << " Canceling the action call.");
          planner_active = false;
        }
        break;

      case AbstractPlannerExecution::STARTED:ROS_DEBUG_STREAM("[Planner Action]: planner state: started");
        break;

      case AbstractPlannerExecution::STOPPED:ROS_DEBUG_STREAM("[Planner Action]: planner state: stopped");
        ROS_WARN_STREAM("[Planner Action]: Planning has been stopped rigorously!");
        result.outcome = mw_msgs::GetPathResult::STOPPED;
        result.message = "Global planner has been stopped!";
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::CANCELED:ROS_DEBUG_STREAM("[Planner Action]: planner state: canceled");
        ROS_DEBUG_STREAM("[Planner Action]: Global planner has been canceled successfully");
        result.path.header.stamp = ros::Time::now();
        result.outcome = mw_msgs::GetPathResult::CANCELED;
        result.message = "Global planner has been canceled!";
        goal_handle.setCanceled(result, result.message);
        planner_active = false;
        break;

        // in progress
      case AbstractPlannerExecution::PLANNING:
        if (execution.isPatienceExceeded()) {
          ROS_INFO_STREAM("[Planner Action]: Global planner patience has been exceeded! Cancel planning...");
          if (!execution.cancel()) {
            ROS_WARN_STREAM_THROTTLE(2.0,
                                     "[Planner Action]: Cancel planning failed or is not supported; must wait until current plan finish!");
            execution.stop(); // try to interrupt planning.
          }
        } else {
          ROS_DEBUG_THROTTLE(2.0, "[Planner Action]: planner state: planning");
        }
        break;

        // found a new plan
      case AbstractPlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result.path.header.stamp = ros::Time::now();
        double cost;
        execution.getPlan(plan, cost);

        ROS_DEBUG_STREAM("[Planner Action]: planner state: found plan with cost: " << execution.getCost());

        if (!transformPlanToGlobalFrame(plan, global_plan)) {
          result.outcome = mw_msgs::GetPathResult::TF_ERROR;
          result.message = "Could not transform the plan to the global frame!";

          ROS_ERROR_STREAM("[Planner Action]: " << result.message << " Canceling the action call.");
          goal_handle.setAborted(result, result.message);
          planner_active = false;
          break;
        }

        if (global_plan.empty()) {
          result.outcome = mw_msgs::GetPathResult::EMPTY_PATH;
          result.message = "Global planner returned an empty path!";

          ROS_ERROR_STREAM("[Planner Action]: " << result.message);
          goal_handle.setAborted(result, result.message);
          planner_active = false;
          break;
        }

        result.path.poses = global_plan;
        result.cost = execution.getCost();
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setSucceeded(result, result.message);

        planner_active = false;
        break;

        // no plan found
      case AbstractPlannerExecution::NO_PLAN_FOUND:ROS_DEBUG_STREAM("[Planner Action]: "
                                                                                << "planner state: no plan found");
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::MAX_RETRIES:ROS_DEBUG_STREAM("[Planner Action]: "
                                                                              << "Global planner reached the maximum number of retries");
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::PAT_EXCEEDED:ROS_DEBUG_STREAM("[Planner Action]: "
                                                                               << "Global planner exceeded the patience time");
        result.outcome = mw_msgs::GetPathResult::PAT_EXCEEDED;
        result.message = "Global planner exceeded the patience time";
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::INTERNAL_ERROR:ROS_FATAL_STREAM("[Planner Action]: "
                                                                                 << "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from planning
        planner_active = false;
        result.outcome = mw_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.setAborted(result, result.message);
        break;

      default:result.outcome = mw_msgs::GetPathResult::INTERNAL_ERROR;
        std::ostringstream ss;
        ss << "Internal error: Unknown state in a planner execution with the number: "
           << static_cast<int>(state_planning_input);
        result.message = ss.str();
        ROS_FATAL_STREAM("[Planner Action]: " << result.message);
        goal_handle.setAborted(result, result.message);
        planner_active = false;
    }

    if (planner_active) {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }
  } // while (planner_active && ros::ok())

  if (!planner_active) {
    ROS_DEBUG_STREAM("[Planner Action]: \"" << name_ << "\" action ended properly.");
  } else {
    ROS_ERROR_STREAM("[Planner Action]: \"" << name_ << "\" action has been stopped!");
  }
}

bool PlannerAction::transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped> &plan,
                                               std::vector<geometry_msgs::PoseStamped> &global_plan) {
  global_plan.clear();
  std::vector<geometry_msgs::PoseStamped>::iterator iter;
  bool tf_success;
  for (iter = plan.begin(); iter != plan.end(); ++iter) {
    geometry_msgs::PoseStamped global_pose;
    tf_success =
        mw_core::transformPose(robot_info_.getTransformListener(), robot_info_.getGlobalFrame(), iter->header.stamp,
                               robot_info_.getTfTimeout(), *iter, robot_info_.getGlobalFrame(), global_pose);
    if (!tf_success) {
      ROS_ERROR_STREAM(
          "[Planner Action]: Can not transform pose from the \"" << iter->header.frame_id << "\" frame into the \""
                                                                 << robot_info_.getGlobalFrame() << "\" frame !");
      return false;
    }
    global_plan.push_back(global_pose);
  }
  return true;
}

} // namespace global_planner