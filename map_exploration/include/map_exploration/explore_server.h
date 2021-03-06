#ifndef MAP_EXPLORATION__EXPLORE_SERVER_H_
#define MAP_EXPLORATION__EXPLORE_SERVER_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <stack>

#include <ros/ros.h>
#include <mw_core/types.h>
#include <mw_core/utility_functions.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <mw_msgs/WaypointNavigationAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>

#include <map_exploration/costmap_client.h>
#include <map_exploration/frontier_search.h>

namespace map_exploration {
/**
 * @class ExploreServer
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class ExploreServer {

 public:

  explicit ExploreServer(const TFPtr &tf_listener_ptr);
  ~ExploreServer();

  void start();
  void stop();

 private:
  /**
   * @brief  Make a global plan
   */
  void makePlan(const ros::TimerEvent &event);

  /**
   * @brief  Publish a frontiers as markers
   */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier> &frontiers);

  void reachedGoal(const actionlib::SimpleClientGoalState &status,
                   const mw_msgs::WaypointNavigationResultConstPtr &result,
                   const geometry_msgs::Point &frontier_goal);

  bool findInBlacklist(const geometry_msgs::Point &goal);

  void sendGoal(const geometry_msgs::Point &target);

  bool stopCallback(std_srvs::Trigger::Request  &req,
                    std_srvs::Trigger::Response &res);

  void reachedGoal(const actionlib::SimpleClientGoalState &status,
                   const mw_msgs::WaypointNavigationResultConstPtr &result);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;

  ros::Publisher marker_array_publisher_;
  ros::ServiceServer stop_server_;

  TFPtr tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<mw_msgs::WaypointNavigationAction> move_base_client_;
  ros::Timer exploring_timer_;

  frontier_exploration::FrontierSearch search_;
  std::vector<frontier_exploration::Frontier> frontier_list_;
  std::vector<geometry_msgs::Point> frontier_blacklist_;
  std::stack<geometry_msgs::Point> traceback_stack_;

  geometry_msgs::Pose home_pose_;
  bool is_goal_active_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  bool visualize_;
  int retries_;
  bool reached_home_;
};

}

#endif  // MAP_EXPLORATION__EXPLORE_SERVER_H_
