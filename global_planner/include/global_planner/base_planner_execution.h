/**
 * 
 * @file base_planner_execution.h
 * @brief The base planner execution class. Every planner execution must inherit this class.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <global_planner/base_planner.h>

#include <mw_core/types.h>
#include <mw_core/utility_functions.h>
#include <mw_core/abstract_classes/abstract_execution_base.h>

namespace global_planner {
/**
 * @brief The AbstractPlannerExecution class loads and binds the global planner plugin. It contains a thread running
 *        the plugin in a cycle to plan and re-plan. An internal state is saved and will be pulled by the server, which
 *        controls the global planner execution. Due to a state change it wakes up all threads connected to the
 *        condition variable.
 */

class AbstractPlannerExecution : public mw_core::AbstractExecution {
 public:
  typedef boost::shared_ptr<::global_planner::AbstractPlannerExecution> Ptr;

  /**
   * @brief Internal states
   */
  enum PlanningState {
    INITIALIZED,   // Planner initialized.
    STARTED,       // Planner started.
    PLANNING,      // Executing the plugin.
    FOUND_PLAN,    // Found a valid plan.
    MAX_RETRIES,   // Exceeded the maximum number of retries without a valid command.
    PAT_EXCEEDED,  // Exceeded the patience time without a valid command.
    NO_PLAN_FOUND, // No plan has been found (MAX_RETRIES and PAT_EXCEEDED are 0).
    CANCELED,      // The planner has been canceled.
    STOPPED,       // The planner has been stopped.
    INTERNAL_ERROR // An internal error occurred.
  };

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   */
  AbstractPlannerExecution();

  /**
   * @brief Destructor
   */
  ~AbstractPlannerExecution();

  /**
   * @brief initialize
   */
  virtual void initialize(const std::string &name,
                          const global_planner::AbstractPlanner::Ptr &planner_ptr);

  /**
   * @brief Returns the last time a valid plan was available.
   * @return time of last valid plan.
   */
  ros::Time getLastValidPlanTime();

  /**
   * @brief Checks whether the patience was exceeded.
   * @return true, if the patience duration was exceeded.
   */
  bool isPatienceExceeded();

  /**
   * @brief Returns the current internal state
   * @return the current internal state
   */
  PlanningState getState();

  /**
   * @brief Gets planning frequency
   */
  double getFrequency() { return frequency_; };

  /**
   * @brief Returns computed cost of the path
   * @return The costs of the computed path
   */
  double getCost();

  /**
   * @brief Cancel the planner execution. This cancel the execution of the thread.
   * @return true, if the planner plugin thread has been stopped
   * @todo Rethink this one
   */
  bool cancel() override;
  //Run before calling the run method
  void preRun() override;
  //Run after execution of the run method
  void postRun() override;

  /**
   * @brief Sets a new goal pose for the planner execution
   * @param goal the new goal pose
   * @param tolerance tolerance to the goal for the planning
   * @todo rethink how would this work
   */
  void setNewGoal(const geometry_msgs::PoseStamped &goal, double tolerance);

  /**
   * @brief Sets a new start pose for the planner execution
   * @param start new start pose
   * @todo rethink how this would work
   */
  void setNewStart(const geometry_msgs::PoseStamped &start);

  /**
   * @brief Sets a new star and goal pose for the planner execution
   * @param start new start pose
   * @param goal new goal pose
   * @param tolerance tolerance to the new goal for the planning
   * @todo rethink how would this work
   */
  void setNewStartAndGoal(const geometry_msgs::PoseStamped &start,
                          const geometry_msgs::PoseStamped &goal,
                          double tolerance);

  /**
   * @brief Starts the planner execution thread with the given parameters.
   * @param start start pose for the planning
   * @param goal goal pose for the planning
   * @param tolerance tolerance to the goal pose for the planning
   * @return true, if the planner thread has been started, false if the thread is already running.
   */
  bool start(const geometry_msgs::PoseStamped &start,
             const geometry_msgs::PoseStamped &goal,
             double tolerance);

  /**
   * @brief Starts the planner execution thread with the given parameters.
   * @param waypoints Waypoints for the planning
   * @param tolerance tolerance to the goal pose for the planning
   * @return true, if the planner thread has been started, false if the thread is already running.
   */
  bool start(const std::vector<geometry_msgs::PoseStamped> &waypoints,
             double tolerance);

  /**
   * @brief Returns a new plan, if one is available.
   * @param plan A reference to a plan, which then will be filled.
   * @param cost A reference to the costs, which then will be filled.
   */
  void getPlan(std::vector<geometry_msgs::PoseStamped> &plan, double &cost);

  bool reset();

 protected:
  /**
   * @brief The main run method, a thread will execute this method. It contains the main planner execution loop.
   */
  void run();

  /**
   * @brief Sets the internal state
   * @param state the current state
   */
  void setState(PlanningState state);

  global_planner::AbstractPlanner::Ptr planner_;
  std::string plugin_name_;

 private:
  boost::mutex state_mtx_;      // mutex to handle safe thread communication for the current state
  boost::mutex plan_mtx_;       // mutex to handle safe thread communication for the plan and plan-costs
  boost::mutex goal_start_mtx_; // mutex to handle safe thread communication for the goal and start pose and waypoints.
  boost::mutex planning_mtx_;   // mutex to handle safe thread communication for the planning_ flag.

  bool has_new_goal_;  // true, if a new goal pose has been set, until it is used.
  bool has_new_start_; // true, if a new start pose has been set, until it is used.

  ros::Time last_call_start_time_; // the last call start time, updated each cycle.
  ros::Time last_valid_plan_time_; // the last time a valid plan has been computed.

  std::vector<geometry_msgs::PoseStamped> plan_; // current global plan
  double cost_;                                  // current global plan cost

  geometry_msgs::PoseStamped start_;                  // the current start pose used for planning
  geometry_msgs::PoseStamped goal_;                   // the current goal pose used for planning
  std::vector<geometry_msgs::PoseStamped> waypoints_; // waypoint array

  double tolerance_;         // optional goal tolerance, in meters
  double
      frequency_;         // planning cycle frequency (used only when running full navigation; we store here for grouping parameters nicely)
  ros::Duration patience_;   // planning patience duration time
  int max_retries_;          // planning max retries
  bool planning_;            // main cycle variable of the execution loop
  std::string robot_frame_;  // robot frame used for computing the current robot pose
  std::string global_frame_; // the global frame in which the planner needs to plan
  PlanningState state_;      // current internal state
};
} // namespace global_planner