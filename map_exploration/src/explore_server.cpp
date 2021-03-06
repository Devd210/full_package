/**
 *
 * @file explore_server.cpp
 * @brief The main exploration server.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include <map_exploration/explore_server.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point &one,
                              const geometry_msgs::Point &two) {
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace map_exploration {

ExploreServer::ExploreServer(const TFPtr &tf_listener_ptr)
    : private_nh_("~"),
      tf_listener_(tf_listener_ptr),
      costmap_client_(private_nh_, relative_nh_, tf_listener_ptr),
      move_base_client_("/mission_executive/waypoint_navigation"),
      last_markers_count_(0),
      is_goal_active_(false),
      reached_home_(false) {

  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("retries", retries_, 10);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  stop_server_ = private_nh_.advertiseService("stop_exploration",
                                              &ExploreServer::stopCallback, this);

  ROS_WARN("[Explore Server]: Waiting to connect to waypoint navigation server");
  move_base_client_.waitForServer();
  ROS_INFO("[Explore Server]: Connected to waypoint navigation server");

  home_pose_ = costmap_client_.getRobotPose();

  exploring_timer_ = relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                              boost::bind(&ExploreServer::makePlan, this, _1));
}

ExploreServer::~ExploreServer() {
  stop();
}

void ExploreServer::visualizeFrontiers(const std::vector<frontier_exploration::Frontier> &frontiers) {
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("[Explore Server]: visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker> &markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.pose.orientation.w = 1.0;
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = (unsigned char) true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
  double max_cost = frontiers.empty() ? 0. : frontiers.back().cost;
  double diff = max_cost - min_cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto &frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.scale.z = 0.0;
    m.points = frontier.points;
    if (findInBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costlier frontiers will be smaller)
    double scale = std::min(std::max(((max_cost - frontier.cost) / diff) * 0.4, 0.1), 0.4);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void ExploreServer::makePlan(const ros::TimerEvent &event) {

  if (is_goal_active_) {
    ROS_INFO_THROTTLE(5, "[Explore Server]: A goal is already active!");
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();

  // get frontiers sorted according to cost
  frontier_list_ = search_.searchFrom(pose.position);
  ROS_DEBUG("[Explore Server]: found %lu frontiers", frontier_list_.size());

  if (frontier_list_.empty()) {
    ROS_WARN("[Explore Server]: Frontier list is empty");
    if (!traceback_stack_.empty()) {
      ROS_WARN("[Explore Server]: Traceback available, backtracking to last goal!");
      sendGoal(traceback_stack_.top());
      traceback_stack_.pop();
    } else {
      stop();
    }
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontier_list_);
  }

  // find non blacklisted frontier
  auto frontier = std::find_if_not(frontier_list_.begin(), frontier_list_.end(),
                                   [this](const frontier_exploration::Frontier &f) {
                                     return findInBlacklist(f.centroid);
                                   });

  if (frontier == frontier_list_.end()) {
    ROS_WARN("[Explore Server]: Reached end of frontier list");
    if (!traceback_stack_.empty()) {
      ROS_WARN("[Explore Server]: Traceback available, backtracking to last goal!");
      sendGoal(traceback_stack_.top());
      traceback_stack_.pop();
    } else {
      stop();
    }
    return;
  }

  if (!traceback_stack_.empty()) {
    // Push current position to traceback if it's not already on top
    if (mw_core::distance(traceback_stack_.top(), pose.position) > 0.1)
      traceback_stack_.push(pose.position);
  } else {
    // Push current position to traceback if it's empty
    traceback_stack_.push(pose.position);
  }

  sendGoal(frontier->centroid);
}

bool ExploreServer::findInBlacklist(const geometry_msgs::Point &goal) {
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D *costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto &frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void ExploreServer::sendGoal(const geometry_msgs::Point &target) {
  // send goal to move_base if we have something new to pursue
  mw_msgs::WaypointNavigationGoal goal;
  goal.target_pose.pose.position = target;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();

  ROS_INFO("[Explore Server]: Sending goal: (x: %lf, y: %lf)", target.x, target.y);
  move_base_client_.sendGoal(
    goal, [this, target](
      const actionlib::SimpleClientGoalState &status,
      const mw_msgs::WaypointNavigationResultConstPtr &result) {
      reachedGoal(status, result, target);
    }, [this]() {
      ROS_DEBUG("[Explore Server]: Active!");
      is_goal_active_ = true;
    });
}

void ExploreServer::reachedGoal(const actionlib::SimpleClientGoalState &status,
                          const mw_msgs::WaypointNavigationResultConstPtr &,
                          const geometry_msgs::Point &frontier_goal) {
  ROS_INFO("[Explore Server]: Reached goal with status: %s", status.toString().c_str());
  if (status != actionlib::SimpleClientGoalState::SUCCEEDED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_WARN("[Explore Server]: Adding current goal to black list");
  }
  is_goal_active_ = false;
}

void ExploreServer::reachedGoal(const actionlib::SimpleClientGoalState &status,
                                const mw_msgs::WaypointNavigationResultConstPtr &) {
  reached_home_ = status == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void ExploreServer::start() {
  exploring_timer_.start();
}

void ExploreServer::stop() {
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("[Explore Server]: ============= Exploration has FINISHED/ STOPPED ==========.");
}

bool ExploreServer::stopCallback(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res) {
  stop();
  ROS_INFO("[Explore Server]: Going back home!");
  mw_msgs::WaypointNavigationGoal goal;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = home_pose_;

  reached_home_ = false;
  for (int i=0 ; i < retries_; ++i) {
    move_base_client_.sendGoal(goal, boost::bind(&ExploreServer::reachedGoal, this, _1, _2));
    move_base_client_.waitForResult();
    if (reached_home_)
      break;
  }

  res.message = "Reached home!";
  res.success = (unsigned char) true;
  return true;
}

}  // namespace map_exploration

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_exploration");

#ifdef USE_OLD_TF
  ros::NodeHandle nh("~");
  TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
#else
  TFPtr tf_buffer_ptr(new TF(ros::Duration(10)));
  tf2_ros::TransformListener tf_listener(*tf_buffer_ptr);
#endif

  map_exploration::ExploreServer explore(tf_buffer_ptr);

  ros::spin();
  return 0;
}
