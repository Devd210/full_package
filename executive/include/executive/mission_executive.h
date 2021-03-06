/**
* @file mission_executive.h
* @author Sourav Agrawal
* @brief Header file for mission executive
*/
#pragma once

//The required header files
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <mw_msgs/ExePathAction.h>
#include <mw_msgs/GetPathAction.h>
#include <mw_msgs/WaypointNavigationAction.h>
#include <mw_msgs/RecoveryAction.h>

#include <mw_msgs/Route.h>
#include <mw_msgs/GetPlan.h>
#include <mw_msgs/SetPlan.h>
#include <mw_msgs/SetRoute.h>
#include <mw_msgs/GetRoute.h>
#include <mw_msgs/GetRouteStatus.h>
#include <mw_msgs/SetString.h>
#include <mw_msgs/mission_status.h>

#include <mw_core/progress_checker.h>
#include <mw_core/utility_functions.h>
#include <mw_core/types.h>

#include <diagnostic/local_diagnostic.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <deque>

// The mission_executive namespace
namespace mission_executive {

typedef actionlib::SimpleActionServer<mw_msgs::WaypointNavigationAction> ActionServerWaypointNavigation;
typedef boost::shared_ptr<ActionServerWaypointNavigation> ActionServerWaypointNavigationPtr;

class RecoveryHandler {

  struct RecoveryInfo {
    int count = 0;
    bool enable = false;
    ros::Duration timeout{0};
  };

public:
  explicit RecoveryHandler(const ros::NodeHandle & nh) {
    double timeout;

    nh.param("recoveries/clear_costmap", timeout, -1.0);
    if (timeout > 0) {
      clear_costmap.enable = true;
      clear_costmap.timeout = ros::Duration(timeout);
    }

    nh.param("recoveries/rotate", timeout, -1.0);
    if (timeout > 0) {
      rotate.enable = true;
      rotate.timeout = ros::Duration(timeout);
    }

    nh.param("recoveries/back_up", timeout, -1.0);
    if (timeout > 0) {
      back_up.enable = true;
      back_up.timeout = ros::Duration(timeout);
    }

    nh.param("recoveries/stepback_and_steerturn", timeout, -1.0);
    if (timeout > 0) {
      stepback_and_steerturn.enable = true;
      stepback_and_steerturn.timeout = ros::Duration(timeout);
    }
  }

  bool triggerRecovery(double progress_time, std::string & recovery_name) {

    bool trigger = false;

    if (clear_costmap.enable && progress_time > clear_costmap.timeout.toSec() * (clear_costmap.count + 1)) {
      recovery_name = "clear_costmap";
      clear_costmap.count++;
      trigger = true;
    }

    if (rotate.enable && progress_time > rotate.timeout.toSec() * (rotate.count + 1)) {
      recovery_name = "rotate";
      rotate.count++;
      trigger = true;
    }

    if (back_up.enable && progress_time > back_up.timeout.toSec() * (back_up.count + 1)) {
      recovery_name = "back_up";
      back_up.count++;
      trigger = true;
    }

    if (stepback_and_steerturn.enable && progress_time > stepback_and_steerturn.timeout.toSec() * (stepback_and_steerturn.count + 1)) {
      recovery_name = "stepback_and_steerturn";
      stepback_and_steerturn.count++;
      trigger = true;
    }

    return trigger;
  }

  void resetCount() {
    clear_costmap.count = 0;
    rotate.count = 0;
    back_up.count = 0;
    stepback_and_steerturn.count = 0;
  }

private:
  RecoveryInfo clear_costmap;
  RecoveryInfo rotate;
  RecoveryInfo back_up;
  RecoveryInfo stepback_and_steerturn;
};


// The executive class
class MissionExecutive {

  // The public variables and functions
 public:

  // brief: The mission executive enumeration for various states of the mission executive
  enum MissionExecutiveState {
    WAITING,         // Waiting state
    PLANNING,        // Planning state
    CONTROLLING,     // Controlling state
    RECOVERING       // Recovering state
  };

  MissionExecutive(); // The executive class constructor

  ~MissionExecutive(); // The executive class destructor

  /*
  * brief:Processes the feedback received from the controller(Changes state variables)
  *
  * Input Params:
  * feedback: Contains the feedback
  *
  * Output Params:
  * NONE
  */
  void feedbackControllerCB(const mw_msgs::ExePathFeedbackConstPtr &feedback);

  /*
  * brief:Infrorms that the goal has been sent to controller through ROS_INFO
  *
  * Input Params:
  * NONE
  *
  * Output Params:
  * NONE
  */
  void activeControllerCB();

  /*
  * brief:Informs that the controller has finished its job on console
  *
  * Input Params:
  * state: The current state
  * result: The result recieved from the controller
  *
  * Output Params:
  * NONE
  */
  void doneControllerCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::ExePathResultConstPtr &result);

  void activeRecoveryCB();

  void doneRecoveryCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::RecoveryResultConstPtr &result);

  /*
  * brief:The callback function for the set_route service
  *
  * Input Params:
  * req: Contains the request message
  * res: This contains the response to be sent back to the client
  *
  * Output Params:
  * Returns true if service call was successful, false otherwise
  */
  bool setCurrentRoute(mw_msgs::SetRoute::Request &req, mw_msgs::SetRoute::Response &resp);

  /*
  * brief:to add the route number in req to make it more robust currently it is a hack
  *
  * Input Params:
  * req: Contains the request message
  * res: This contains the response to be sent back to the client
  *
  * Output Params:
  * Returns true if service call was successfull,false otherwise
  */
  bool routeStatus(mw_msgs::GetRouteStatus::Request &req, mw_msgs::GetRouteStatus::Response &resp);

  /*
  * brief:The callback function for the set_plan service
  *
  * Input Params:
  * req: Contains the request message
  * res: This contains the response to be sent back to the client
  *
  * Output Params:
  * Returns true if service call was successful, false otherwise
  */
  bool setCurrentPlan(mw_msgs::SetPlan::Request &req, mw_msgs::SetPlan::Response &resp);

  /*
  * brief:Procceses the executive call queue(executive_queue_)
  *
  * Input Params:
  * NONE
  *
  * Output Params:
  * NONE
  */
  void spinClass();

  void executeCB(const ros::TimerEvent &event);

  void donePlannerCB(const actionlib::SimpleClientGoalState &state,
                     const mw_msgs::GetPathResultConstPtr &result);

  void activePlannerCB();

  void rvizGoalCB(const geometry_msgs::PoseStamped::Ptr &msg);

  bool changePlanner(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp);

  bool changeController(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp);

  bool abortPlannerGoals(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  bool abortControllerGoals(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  bool abortMission(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  bool triggerRecovery(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp);

  void callActionWaypointNavigation(const ActionServerWaypointNavigation::GoalConstPtr &goal);
  
  /*
  * brief: get enum values representing state of the mission
  * Input Params : NONE
  * Output Params : NONE
  * Returns enum value MissionExecutiveState for the state of machine
  */
  MissionExecutiveState getMissionExecState();
  
  /*
  * brief:get enum values to string representation for the state of the mission
  * Input Params : NONE
  * Output Params : NONE
  * Returns state of the machine
  */
  std::string getMissionExecStateName();
  
  /*
  * brief:get mission_status message (state, controller type/name, global type/name)
  * Input Params : NONE
  * Output Params : NONE
  * Returns message describing state, controller type/name, global type/name
  */
  mw_msgs::mission_status getMissionStatus();
  
  /*
  * brief: start publisher and publish message mission_status.msg 
  * Input Params : NONE 
  * Output Params : NONE
  * Returns NONE
  */
  void startMissionExecutiveStatusPublisher();

  /**
  * @brief Returns the frequency of the loop
  * @return the frequency of the current loop
  */
  double getLoopFrequency() { return loop_frequency_; }

  // The private variables and functions
 private:

  ros::NodeHandle nh_; // The node handle
  ros::NodeHandle private_nh_;

  //  for the diagnostics
  ros::CallbackQueue callback_queue_; // The callback queue

  ros::Subscriber rviz_goal_sub_; // Subscriber for RViz goals

  ros::Publisher cmd_vel_pub_;   // The cmd_vel publisher
  ros::Publisher plan_pub_; // The plan publisher
  ros::Publisher route_marker_pub_; // The route visualization publisher
  ros::Publisher goal_queue_pub_; // The goal queue visualization publisher
  ros::Publisher mission_executive_status_pub_; // The mission status publisher

  std::string map_frame_; // Stores the map frame id
  std::string robot_frame_; //Stores the robot frame
  double loop_frequency_; // The frequency of the loop

  bool auto_pause_;                      // Variable to keep track of 'pause' or 'resume' navigation/motion of the robot

  nav_msgs::Path global_plan_;           // Stores the global plan i.e. the route

  MissionExecutiveState state_;          // The state of the executive, waiting/controlling/planning

  mw_msgs::Route current_route_;
  // this variable is used to keep track for CEO and goes back to NOT_STARTED once the status of DONE is sent to CEO

  ros::ServiceServer set_route_;                // Server that broadcasts the set_route service
  ros::ServiceServer route_status_;             // Server that broadcasts the route_status service
  ros::ServiceServer set_plan_;                 // Server that broadcasts the set_plan service
  ros::ServiceServer change_planner_;           // Server that allows dynamic planner changes
  ros::ServiceServer change_controller_;        // Server that allows dynamic controller changes
  ros::ServiceServer abort_planner_goals_;      // Server that cancels all planner goals
  ros::ServiceServer abort_controller_goals_;   // Server that cancels all controller goals
  ros::ServiceServer abort_mission_;            // Server that cancels all controller and planner goals
  ros::ServiceServer trigger_recovery_;         // Server that triggers a specific recovery

  std::string cmd_vel_topic_;
  std::string plan_topic_;
  std::string route_topic_;
  std::string goal_queue_topic_;
  std::string mission_status_topic_;

  std::string planner_name_;
  std::string controller_name_;

  bool action_server_running_;
  ActionServerWaypointNavigationPtr action_server_;

  actionlib::SimpleActionClient<mw_msgs::ExePathAction> controller_; // Action client for the controller
  actionlib::SimpleActionClient<mw_msgs::GetPathAction> planner_;
  actionlib::SimpleActionClient<mw_msgs::RecoveryAction> recovery_;

  std::deque<mw_msgs::GetPathGoal> planner_goal_queue_;
  std::deque<mw_msgs::ExePathGoal> controller_goal_queue_;

  mw_msgs::GetPathGoal::Ptr current_planner_goal_;
  mw_msgs::ExePathGoal::Ptr current_controller_goal_;

  geometry_msgs::Twist cmd_vel_;                               // The cmd_vel value
  geometry_msgs::Twist cmd_vel_zero_;                          // The cmd_vel_zero value

  visualization_msgs::MarkerArray goal_queue_marker_msg_;      // Marker array to hold goal queue
  visualization_msgs::Marker goal_text_marker_;                // Marker array to hold text
  visualization_msgs::Marker goal_point_marker_;               // Marker array to hold point

  std::shared_ptr<mw_core::ProgressChecker> progress_checker_;

  bool is_controlling;
  double path_deviation_;                                      // The path deviation value
  double distance_to_goal_;                                    // Indicates the remaining distance to goal
  int error_code_;                                             // Indicates the error code
  std::string message_;                                        // Indicates error message
  ros::Time last_feedback_time_;                               // Represents the last feedback time

  ros::Duration controller_reset_timeout_;
  int controller_reset_tries_;

  ros::Timer execute_timer_;                                   // The timer of execution

  TFPtr tf_listener_ptr_;
  double tf_timeout_;

  geometry_msgs::PoseStamped current_pose_;

  tf2_ros::TransformListener *tf_listener_;

  std::shared_ptr<RecoveryHandler> recovery_handler_;

  double min_distance_tolerance_;
  double min_angular_tolerance_;

  int max_retries_;

  double max_time_lag_;                                        // Max time to check if feedback received from controller
  double decay_;

  void publishRouteMarker(const std::vector<geometry_msgs::PoseStamped> &waypoints);
  void publishGoalQueue();

  std::string recovery_name_;
  void sendRecoveryGoal();

  void clearPlannerQueue();
  void clearControllerQueue();
  void reset();

};
} // namespace mission_executive
