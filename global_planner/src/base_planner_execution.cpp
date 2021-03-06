/**
 * 
 * @file base_planner_execution.cpp
 * @brief The implementation of base planner execution class. Every planner execution must inherit this class.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#include <global_planner/base_planner_execution.h>
#include <tf/transform_datatypes.h>

namespace global_planner {

AbstractPlannerExecution::AbstractPlannerExecution()
    : mw_core::AbstractExecution(),
      planner_(nullptr),
      state_(INITIALIZED),
      planning_(false),
      has_new_start_(false),
      has_new_goal_(false),
      max_retries_(3) {
  ros::NodeHandle private_nh("~");
  private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
  private_nh.param("map_frame", global_frame_, std::string("map"));
}

AbstractPlannerExecution::~AbstractPlannerExecution() {
}

void AbstractPlannerExecution::initialize(const std::string &name,
                                          const global_planner::AbstractPlanner::Ptr &planner_ptr) {
  planner_ = planner_ptr;
  cancel_ = false;
  AbstractExecution::initialize(name);
}

bool AbstractPlannerExecution::reset() {
  setState(INITIALIZED);
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  plan_.clear();
  cancel_ = false;
  has_new_start_ = false;
  has_new_goal_ = false;
  planning_ = false;
  ROS_DEBUG("[Planner Execution]: planner execution was reset");
  return true;
}

double AbstractPlannerExecution::getCost() {
  //ROS_DEBUG("Computing cost for plan of size %ld,locking the mutex now", plan_.size());
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  //ROS_DEBUG("Current cost is %lf",cost_);
  if ((cost_ - 0) < 0.0001 && !plan_.empty()) {
    ROS_DEBUG_STREAM("[Planner Execution]: Compute costs by discrete path length!");
    double cost = 0.0;
    geometry_msgs::PoseStamped prev_pose = plan_.front();
    for (std::vector<geometry_msgs::PoseStamped>::iterator iter = plan_.begin() + 1; iter != plan_.end(); ++iter) {
      cost += mw_core::distance(prev_pose, *iter);
      prev_pose = *iter;
    }
    cost_ = cost;
    return cost;
  }
  return cost_;
}

typename AbstractPlannerExecution::PlanningState AbstractPlannerExecution::getState() {
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return state_;
}

void AbstractPlannerExecution::setState(PlanningState state) {
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  state_ = state;
}

ros::Time AbstractPlannerExecution::getLastValidPlanTime() {
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  return last_valid_plan_time_;
}

bool AbstractPlannerExecution::isPatienceExceeded() {
  return !patience_.isZero() && (ros::Time::now() - last_call_start_time_ > patience_);
}

void AbstractPlannerExecution::getPlan(std::vector<geometry_msgs::PoseStamped> &plan, double &cost) {
  //boost::lock_guard<boost::mutex> guard(plan_mtx_);
  plan_mtx_.lock();
  plan = plan_;
  plan_mtx_.unlock();
  ROS_DEBUG("[Planner Execution]: Getting the cost of the plan");
  cost = getCost();
}

void AbstractPlannerExecution::setNewGoal(const geometry_msgs::PoseStamped &goal, double tolerance) {
  boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
  goal_ = goal;
  waypoints_.clear();
  waypoints_.push_back(goal);
  tolerance_ = tolerance;
  has_new_goal_ = true;
  cancel_ = false;
}

void AbstractPlannerExecution::setNewStart(const geometry_msgs::PoseStamped &start) {
  boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
  start_ = start;
  waypoints_.clear();
  waypoints_.push_back(start);
  has_new_start_ = true;
  cancel_ = false;
}

void AbstractPlannerExecution::setNewStartAndGoal(const geometry_msgs::PoseStamped &start,
                                                  const geometry_msgs::PoseStamped &goal,
                                                  double tolerance) {
  boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
  start_ = start;
  goal_ = goal;
  waypoints_.clear();
  waypoints_.push_back(start);
  waypoints_.push_back(goal);
  tolerance_ = tolerance;
  has_new_start_ = true;
  has_new_goal_ = true;
  cancel_ = false;
}

bool AbstractPlannerExecution::start(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     double tolerance) {
  if (planning_) {
    ROS_DEBUG("[Planner Execution]: Trying to plan when the planner is already active, calling reset!");
    reset();
  }
  boost::lock_guard<boost::mutex> guard(planning_mtx_);
  planning_ = true;
  start_ = start;
  goal_ = goal;
  waypoints_.clear();
  waypoints_.push_back(start);
  waypoints_.push_back(goal);
  tolerance_ = tolerance;
  ROS_DEBUG_STREAM(
      "[Planner Execution]: Start planning from the start pose: (" << start.pose.position.x << ","
                                                                   << start.pose.position.y << ","
                                                                   << start.pose.position.z << ")"
                                                                   << "to the Goal Pose: (" << goal.pose.position.x
                                                                   << ","
                                                                   << goal.pose.position.y << ","
                                                                   << goal.pose.position.z
                                                                   << ")");

  return AbstractExecution::start();
}

bool AbstractPlannerExecution::start(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                                     double tolerance) {
  if (planning_) {
    ROS_DEBUG("[Planner Execution]: Trying to plan when the planner is already active, calling reset!");
    reset();
  }
  boost::lock_guard<boost::mutex> guard(planning_mtx_);
  planning_ = true;
  start_ = waypoints.front();
  waypoints_.clear();
  waypoints_ = waypoints;
  goal_ = waypoints.back();
  tolerance_ = tolerance;
  ROS_DEBUG_STREAM(
      "[Planner Execution]: Start planning from the start pose: (" << start_.pose.position.x << ","
                                                                   << start_.pose.position.y << ","
                                                                   << start_.pose.position.z << ")"
                                                                   << "to the Goal Pose: (" << goal_.pose.position.x
                                                                   << ","
                                                                   << goal_.pose.position.y << ","
                                                                   << goal_.pose.position.z << ")");
  return AbstractExecution::start();
}

bool AbstractPlannerExecution::cancel() {
  cancel_ = true;
  join();
  return true;
}

void AbstractPlannerExecution::postRun() {
  ROS_DEBUG("[Planner Execution]: Resetting the state machine for a new run!");
  reset();
}

void AbstractPlannerExecution::preRun() {
//  reset();
}

void AbstractPlannerExecution::run() {
  boost::lock_guard<boost::mutex> guard(planning_mtx_);
  int retries = 0;

  bool success = false;
  bool make_plan = false;
  bool exceeded = false;

  last_call_start_time_ = ros::Time::now();
  last_valid_plan_time_ = ros::Time::now();

  try {
    while (planning_ && ros::ok()) {
      boost::chrono::thread_clock::time_point start_time = boost::chrono::thread_clock::now();

      std::vector<geometry_msgs::PoseStamped> plan;
      double cost;

      goal_start_mtx_.lock();
      if (has_new_start_) {
        has_new_start_ = false;
        ROS_INFO_STREAM("[Planner Execution]: A new start pose is available. Planning with the new start pose");
        exceeded = false;
        ROS_INFO_STREAM(
            "[Planner Execution]: New planning start pose: (" << start_.pose.position.x << "," << start_.pose.position.y
                                                              << ","
                                                              << start_.pose.position.z << ")");
      }
      if (has_new_goal_) {
        has_new_goal_ = false;
        ROS_INFO_STREAM(
            "[Planner Execution]: A new goal pose is available. Planning with the new goal pose and the tolerance: "
                << tolerance_);
        exceeded = false;
        ROS_INFO_STREAM(
            "[Planner Execution]: New goal pose: (" << goal_.pose.position.x << "," << goal_.pose.position.y << ","
                                                    << goal_.pose.position.z
                                                    << ")");
      }
      make_plan = !(success || exceeded) || has_new_start_ || has_new_goal_;
      goal_start_mtx_.unlock();

      setState(PLANNING);
      if (make_plan) {
        ROS_DEBUG("[Planner Execution]: Making a plan now");
        plan.clear();
        ROS_DEBUG("[Planner Execution]: Trying to plan with %ld waypoints", waypoints_.size());
        outcome_ = planner_->makePlan(waypoints_, tolerance_, plan, cost, message_);
        ROS_DEBUG("[Planner Execution]: The message is %s", message_.c_str());
        ROS_DEBUG("[Planner Execution]: The outcome of the plan was %d", outcome_);
        ROS_DEBUG("[Planner Execution]: The size of the plan is %ld", plan.size());
        success = outcome_ < 10;

        if (cancel_ && !isPatienceExceeded()) {
          setState(CANCELED);
          ROS_INFO_STREAM("[Planner Execution]: The global planner has been canceled with patience check!");
          planning_ = false;
          condition_.notify_all();
        } else if (success) {
          ROS_DEBUG("[Planner Execution]: Successfully found a plan");
          exceeded = false;
          planning_ = false;

          plan_mtx_.lock();
          plan_ = plan;
          cost_ = cost;
          last_valid_plan_time_ = ros::Time::now();
          plan_mtx_.unlock();
          setState(FOUND_PLAN);
          condition_.notify_all();
        } else if (max_retries_ >= 0 && ++retries > max_retries_) {
          ROS_INFO_STREAM("[Planner Execution]: Planning reached max retries! (" << max_retries_ << ")");
          setState(MAX_RETRIES);
          exceeded = true;
          planning_ = false;
          condition_.notify_all();
        } else if (isPatienceExceeded()) {
          ROS_INFO_STREAM("[Planner Execution]: Planning patience (" << patience_.toSec() << "s) has been exceeded"
                                                                     << (cancel_ ? "; planner canceled!" : ""));
          setState(PAT_EXCEEDED);
          exceeded = true;
          planning_ = false;
          condition_.notify_all();
        } else if (max_retries_ == 0 && patience_.isZero()) {
          ROS_INFO_STREAM("[Planner Execution]: Planning could not find a plan!");
          exceeded = true;
          setState(NO_PLAN_FOUND);
          planning_ = false;
          condition_.notify_all();
        } else {
          exceeded = false;
          ROS_DEBUG("[Planner Execution]: Planning could not find a plan! Trying again...");
        }
      } else if (cancel_) {
        ROS_INFO("[Planner Execution]: The global planner has been canceled!");
        setState(CANCELED);
        planning_ = false;
        condition_.notify_all();
      }
    }
    ROS_DEBUG("[Planner Execution]: Out of while loop");
  }
  catch (const boost::thread_interrupted &ex) {
    ROS_WARN("[Planner Execution]: Planner thread interrupted!");
    setState(STOPPED);
    planning_ = false;
    condition_.notify_all();
  }
  catch (...) {
    ROS_FATAL_STREAM(
        "[Planner Execution]: Unknown error occurred: " << boost::current_exception_diagnostic_information());
    setState(INTERNAL_ERROR);
  }
  ROS_DEBUG("[Planner Execution]: Planner execution run has exited");
}

} // namespace global_planner
