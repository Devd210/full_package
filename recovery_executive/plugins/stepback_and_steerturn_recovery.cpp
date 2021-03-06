/**
 *
 * @file stepback_and_steerturn_recovery.cpp
 * @brief stepback and steerturn recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include <recovery_executive/plugins/stepback_and_steerturn_recovery.h>
#include <pluginlib/class_list_macros.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(recovery::StepBackAndSteerTurnRecovery, recovery::AbstractRecovery)

namespace recovery {

StepBackAndSteerTurnRecovery::StepBackAndSteerTurnRecovery () :
  global_costmap_(nullptr), local_costmap_(nullptr), initialized_(false), cancel_(false)
{
  TWIST_STOP.linear.x = 0.0;
  TWIST_STOP.linear.y = 0.0;
  TWIST_STOP.linear.z = 0.0;
  TWIST_STOP.angular.x = 0.0;
  TWIST_STOP.angular.y = 0.0;
  TWIST_STOP.angular.z = 0.0;
}

StepBackAndSteerTurnRecovery::~StepBackAndSteerTurnRecovery ()
{
  delete world_model_;
}

void StepBackAndSteerTurnRecovery::initialize(const std::string &name, const TFPtr &tf_listener_ptr) {
  ROS_ASSERT(!initialized_);
  name_ = name;
  tf_listener_ptr_ = tf_listener_ptr;

  ros::NodeHandle nh;
  local_costmap_ = std::make_shared<costmap_2d::CostmapHelper>(nh, "local_costmap");
  global_costmap_ = std::make_shared<costmap_2d::CostmapHelper>(nh, "global_costmap");

  while ((!local_costmap_->isReady() || !global_costmap_->isReady()) && ros::ok()) {
    ROS_WARN("[SBST Recovery]: Waiting for initial costmap data fetch...");
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  ROS_INFO("[SBST Recovery]: Initial costmap data fetch successful!");

  global_frame_ = local_costmap_->getGlobalFrameID();
  robot_base_frame_ = "base_link";

  world_model_ = new trajectory_planner::CostmapModel(local_costmap_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  recover_run_pub_ = nh_.advertise<std_msgs::Bool>("recover_run", 10);

  ros::NodeHandle private_nh("~/" + name);

  private_nh.param("duration", duration_, 1.0);
  private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
  private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
  private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
  private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("simulation_frequency", simulation_frequency_, 5.0);
  private_nh.param("simulation_inc", simulation_inc_, 1/simulation_frequency_);

  private_nh.param("only_single_steering", only_single_steering_, true);
  private_nh.param("trial_times", trial_times_, 5);
  private_nh.param("obstacle_patience", obstacle_patience_, 0.5);
  private_nh.param("obstacle_check_frequency", obstacle_check_frequency_, 5.0);
  private_nh.param("sim_angle_resolution", sim_angle_resolution_, 0.1);

  // back
  private_nh.param("linear_vel_back", linear_vel_back_, -0.3);
  private_nh.param("step_back_length", step_back_length_, 1.0);
  private_nh.param("step_back_timeout", step_back_timeout_, 15.0);
  //-- steer
  private_nh.param("linear_vel_steer", linear_vel_steer_, 0.3);
  private_nh.param("angular_speed_steer", angular_speed_steer_, 0.5);
  private_nh.param("turn_angle", turn_angle_, 2.0);
  private_nh.param("steering_timeout", steering_timeout_, 15.0);
  //-- forward
  private_nh.param("linear_vel_forward", linear_vel_forward_, 0.3);
  private_nh.param("step_forward_length", step_forward_length_, 0.5);
  private_nh.param("step_forward_timeout", step_forward_timeout_, 15.0);

  initialized_ = true;
  cancel_ = false;
}

geometry_msgs::Twist scaleTwist(const geometry_msgs::Twist& twist, const double scale) {
  geometry_msgs::Twist t;
  t.linear.x = twist.linear.x * scale;
  t.linear.y = twist.linear.y * scale;
  t.angular.z = twist.angular.z * scale;
  return t;
}

geometry_msgs::Pose2D forwardSimulate(const geometry_msgs::Pose2D& p, const geometry_msgs::Twist& twist, const double t=1.0) {
  geometry_msgs::Pose2D p2;
  const double linear_vel = twist.linear.x;
  p2.theta = p.theta + twist.angular.z;//*t;
  p2.x = p.x + linear_vel * cos(p2.theta)*t;
  p2.y = p.y + linear_vel * sin(p2.theta)*t;
  return p2;
}

/// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
double StepBackAndSteerTurnRecovery::normalizedPoseCost(const geometry_msgs::Pose2D& pose) const
{
  geometry_msgs::Point p;
  p.x = pose.x;
  p.y = pose.y;

  unsigned int pose_map_idx_x, pose_map_idx_y;
  // convert point unit from [m] to [idx]
  if (!local_costmap_->worldToMap(p.x, p.y, pose_map_idx_x, pose_map_idx_y))
    return 1e9;
  ROS_DEBUG_NAMED ("top", "Trying to get cost at (%d, %d) in getCost", pose_map_idx_x, pose_map_idx_y);
  const double c = local_costmap_->getCost(pose_map_idx_x, pose_map_idx_y);

  return c < 0 ? 1e9 : c;
}


/// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
/// d seconds if we follow twist
/// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
/// the first k of those d seconds, but this is not done
geometry_msgs::Pose2D StepBackAndSteerTurnRecovery::getPoseToObstacle (const geometry_msgs::Pose2D& current, const geometry_msgs::Twist& twist) const
{
  double cost = 0;
  cost = normalizedPoseCost(current);
  double t; // Will hold the first time that is invalid
  geometry_msgs::Pose2D current_tmp = current;
  double next_cost;

  ROS_DEBUG_NAMED ("top", " ");
  for (t=simulation_inc_; t<=duration_ + 500; t+=simulation_inc_) {
    ROS_DEBUG_NAMED ("top", "start loop");
    current_tmp = forwardSimulate(current, twist, t);
    ROS_DEBUG_NAMED ("top", "finish fowardSimulate");
    next_cost = normalizedPoseCost(current_tmp);
    ROS_DEBUG_NAMED ("top", "finish Cost");
    //if (next_cost > cost) {
    if (/*next_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||*/ next_cost == costmap_2d::LETHAL_OBSTACLE) {
      ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t << " and pose " << forwardSimulate(current, twist, t)
                                                 << " is " << next_cost << " which is greater than previous cost " << cost);
      break;
    }
    cost = next_cost;
  }
  ROS_DEBUG_NAMED ("top", "cost = %.2f, next_cost = %.2f", cost, next_cost);
  ROS_DEBUG_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist.linear.x, twist.angular.z);
  ROS_DEBUG_NAMED ("top", "init = (%.2f, %.2f, %.2f), current = (%.2f, %.2f, %.2f)",
                   current.x, current.y, current.theta, current_tmp.x, current_tmp.y, current_tmp.theta);
  ROS_DEBUG_NAMED ("top", "time = %.2f", t);

  // return t-simulation_inc_;
  return current_tmp;
}

double linearSpeed (const geometry_msgs::Twist& twist) {
  return sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
}

double angularSpeed (const geometry_msgs::Twist& twist) {
  return fabs(twist.angular.z);
}

// Scale twist so we can stop in the given time, and so it's within the max velocity
geometry_msgs::Twist StepBackAndSteerTurnRecovery::scaleGivenAccelerationLimits (const geometry_msgs::Twist& twist, const double time_remaining) const
{
  const double linear_speed = linearSpeed(twist);
  const double angular_speed = angularSpeed(twist);
  const double linear_acc_scaling = linear_speed/(time_remaining*linear_acceleration_limit_);
  const double angular_acc_scaling = angular_speed/(time_remaining*angular_acceleration_limit_);
  const double acc_scaling = std::max(linear_acc_scaling, angular_acc_scaling);
  const double linear_vel_scaling = linear_speed/linear_speed_limit_;
  const double angular_vel_scaling = angular_speed/angular_speed_limit_;
  const double vel_scaling = std::max(linear_vel_scaling, angular_vel_scaling);
  return scaleTwist(twist, std::max(1.0, std::max(acc_scaling, vel_scaling)));
}

// Get pose in local costmap framoe
geometry_msgs::Pose2D StepBackAndSteerTurnRecovery::getCurrentLocalPose () const
{
  geometry_msgs::PoseStamped global_pose;
  if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_base_frame_,
                             global_frame_, ros::Duration(1.0), global_pose)) {
    ROS_ERROR("[SBST Recovery]: Could not get robot pose");
    throw TFException("Could not get robot pose");
  }

  geometry_msgs::Pose2D pose;
  pose.x = global_pose.pose.position.x;
  pose.y = global_pose.pose.position.y;
  pose.theta = tf2::getYaw(global_pose.pose.orientation);
  return pose;
}

void StepBackAndSteerTurnRecovery::moveSpacifiedLength (const geometry_msgs::Twist twist, const double distination, const COSTMAP_SEARCH_MODE mode) const
{
  double distination_cmd = distination;
  double min_dist_to_obstacle = getMinimalDistanceToObstacle(mode);

  std::string mode_name;
  double time_out;

  switch (mode) {
    case FORWARD:
      mode_name = "FORWARD";
      time_out = step_forward_timeout_;
      if (min_dist_to_obstacle < distination)
      {
        distination_cmd = min_dist_to_obstacle - obstacle_patience_;

        ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
        ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
        ROS_WARN_NAMED ("top", "moving length is switched from %.2f [m] to %.2f in %s", distination, distination_cmd,mode_name.c_str());
      }
      break;
    case FORWARD_LEFT:
      mode_name = "FORWARD_LEFT";
      time_out = steering_timeout_;
      if (min_dist_to_obstacle < obstacle_patience_)
      {
        distination_cmd = 0.0;

        ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
        ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
        ROS_WARN_NAMED ("top", "stop turning because an obstacle is too close in %s", mode_name.c_str());
      }
      break;
    case FORWARD_RIGHT:
      mode_name = "FORWARD_RIGHT";
      time_out = steering_timeout_;
      if (min_dist_to_obstacle < obstacle_patience_)
      {
        distination_cmd = 0.0;

        ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
        ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
        ROS_WARN_NAMED ("top", "stop turning because an obstacle is too close in %s", mode_name.c_str());
      }
      break;
    case BACKWARD:
      mode_name = "BACKWARD";
      time_out = step_back_timeout_;
      if (min_dist_to_obstacle < distination)
      {
        distination_cmd = min_dist_to_obstacle - 2 * obstacle_patience_;

        ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
        ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
        ROS_WARN_NAMED ("top", "moving length is switched from %.2f [m] to %.2f in %s", distination, distination_cmd, mode_name.c_str());
      }
      break;
    default:
      break;
  }

  const double frequency = 5.0;
  ros::Rate r(frequency);

  const geometry_msgs::Pose2D initialPose = getCurrentLocalPose();

  int log_cnt = 0;
  int log_frequency = (int)obstacle_check_frequency_;

  ros::Time time_begin = ros::Time::now();
  while (double dist_diff = getCurrentDistDiff(initialPose, distination_cmd, mode) > 0.01)
  {
    double remaining_time = dist_diff / base_frame_twist_.linear.x;
    double min_dist = getMinimalDistanceToObstacle(mode);

    // time out
    if(time_out > 0.0 &&
      time_begin + ros::Duration(time_out) < ros::Time::now())
    {
      //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, remaining_time));
      cmd_vel_pub_.publish(TWIST_STOP);
      ROS_WARN_NAMED ("top", "time out at %s", mode_name.c_str());
      ROS_WARN_NAMED ("top", "%.2f [sec] elapsed.", time_out);
      break;
    }

    // detect an obstacle
    if(min_dist < obstacle_patience_)
    {
      //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, remaining_time));
      cmd_vel_pub_.publish(TWIST_STOP);
      ROS_WARN_NAMED ("top", "obstacle detected at %s", mode_name.c_str());
      ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist, mode_name.c_str());
      break;
    }

    //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(twist, remaining_time));
    cmd_vel_pub_.publish(twist);
    if(log_cnt++ % log_frequency == 0)
    {
      ROS_DEBUG_NAMED ("top", "no obstacle around");
      ROS_INFO_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist, mode_name.c_str());
    }

    ros::spinOnce();
    r.sleep();
  }
}

double StepBackAndSteerTurnRecovery::getCurrentDiff(const geometry_msgs::Pose2D initialPose, const COSTMAP_SEARCH_MODE mode) const
{

  const geometry_msgs::Pose2D& currentPose = getCurrentLocalPose();
  ROS_DEBUG_NAMED ("top", "current pose (%.2f, %.2f, %.2f)", currentPose.x,
                   currentPose.y, currentPose.theta);

  double current_diff;

  switch (mode) {
    case FORWARD:
    case BACKWARD:
      current_diff = getDistBetweenTwoPoints(currentPose, initialPose);
      ROS_DEBUG_NAMED ("top", "current_diff in translation = %.2f", current_diff);
      break;
    case FORWARD_LEFT:
    case FORWARD_RIGHT:
      current_diff = initialPose.theta - currentPose.theta;
      current_diff = fabs(current_diff);
      ROS_DEBUG_NAMED ("top", "initialPose.Theta = %.2f, currentPose.theta = %.2f", initialPose.theta, currentPose.theta);
      ROS_DEBUG_NAMED ("top", "current_diff in angle = %.2f", current_diff);
    default:
      break;
  }

  return current_diff;
}

double StepBackAndSteerTurnRecovery::getCurrentDistDiff(const geometry_msgs::Pose2D initialPose, const double distination, COSTMAP_SEARCH_MODE mode) const
{
  const double dist_diff = distination - getCurrentDiff(initialPose, mode);
  ROS_DEBUG_NAMED ("top", "dist_diff = %.2f", dist_diff);

  return dist_diff;
}

double StepBackAndSteerTurnRecovery::getMinimalDistanceToObstacle(const COSTMAP_SEARCH_MODE mode) const
{
  double max_angle = 0.0, min_angle = 0.0;
  geometry_msgs::Twist twist = TWIST_STOP;

  switch (mode) {
    case FORWARD:
      twist.linear.x = linear_vel_forward_;
      max_angle = M_PI/3.0;
      min_angle = -M_PI/3.0;
      break;
    case FORWARD_LEFT:
      twist.linear.x = linear_vel_forward_;
      max_angle = M_PI/2.0;
      min_angle = 0.0;
      break;
    case FORWARD_RIGHT:
      twist.linear.x = linear_vel_forward_;
      max_angle = 0.0;
      min_angle = -M_PI/2.0;
      break;
    case BACKWARD:
      twist.linear.x = linear_vel_back_;
      max_angle = M_PI/3.0;
      min_angle = -M_PI/3.0;
      break;
    default:
      break;
  }

  const geometry_msgs::Pose2D& current = getCurrentLocalPose();
  double min_dist = INFINITY;

  for(double angle = min_angle; angle < max_angle; angle+=sim_angle_resolution_)
  {
    twist.angular.z = angle;
    geometry_msgs::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
    double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

    if(dist_to_obstacle < min_dist)
      min_dist = dist_to_obstacle;
  }

  ROS_DEBUG_NAMED ("top", "min_dist = %.2f", min_dist);

  return min_dist;
}

int StepBackAndSteerTurnRecovery::determineTurnDirection()
{
  // simulate and evaluate cost
  const geometry_msgs::Pose2D& current = getCurrentLocalPose();

  geometry_msgs::Twist twist = TWIST_STOP;
  twist.linear.x = linear_vel_forward_;

  std::vector<double> dist_to_obstacle_r;
  std::vector<double> dist_to_obstacle_l;
  double max = M_PI/2.0;
  double min = - max;
  for(double angle = min; angle < max; angle += sim_angle_resolution_)
  {
    twist.angular.z = angle;
    geometry_msgs::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
    double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

    ROS_DEBUG_NAMED ("top", "(%.2f, %.2f, %.2f) for %.2f [m] to obstacle",
                     twist.linear.x, twist.linear.y, twist.angular.z, dist_to_obstacle);

    if(angle > 0.0)
      dist_to_obstacle_l.push_back(dist_to_obstacle);
    else if(angle < 0.0)
      dist_to_obstacle_r.push_back(dist_to_obstacle);
    else
      ;// do nothing
  }

  // determine the directoin to go from cost
  /*
  double sum_l = 0.0;
  double sum_r = 0.0;
  double ave_l = 0.0;
  double ave_r = 0.0;
  for(int i = 0; i < dist_to_obstacle_l.size(); i++)
      sum_l += dist_to_obstacle_l[i];
  for(int i = 0; i < dist_to_obstacle_r.size(); i++)
      sum_r += dist_to_obstacle_r[i];
  ave_l = sum_l / dist_to_obstacle_l.size();
  ave_r = sum_r / dist_to_obstacle_r.size();
  ROS_DEBUG_NAMED ("top", "sum_l = %.2f, sum_r = %.2f", sum_l, sum_r);
  ROS_DEBUG_NAMED ("top", "size_l = %d, size_r = %d", (int)dist_to_obstacle_l.size(), (int)dist_to_obstacle_r.size());
  ROS_DEBUG_NAMED ("top", "ave_l = %.2f, ave_r = %.2f", ave_l, ave_r);
  */

  double min_l = *min_element(dist_to_obstacle_l.begin(), dist_to_obstacle_l.end());
  double min_r = *min_element(dist_to_obstacle_r.begin(), dist_to_obstacle_r.end());
  ROS_INFO_NAMED ("top", "min_l = %.2f [m], min_r = %.2f [m]", min_l, min_r);

  int ret_val;

  if(min_l < min_r)
    ret_val = RIGHT; // if obstacle settles on left, turn right
  else
    ret_val = LEFT; // vice versa

  return ret_val;
}

double StepBackAndSteerTurnRecovery::getDistBetweenTwoPoints(const geometry_msgs::Pose2D pose1, const geometry_msgs::Pose2D pose2) const
{
  double dist_to_obstacle = (pose1.x - pose2.x) * (pose1.x - pose2.x) +
    (pose1.y - pose2.y) * (pose1.y - pose2.y);
  return sqrt(dist_to_obstacle);
}

uint32_t StepBackAndSteerTurnRecovery::runBehavior(std::string &message) {
  ROS_ASSERT (initialized_);

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "********Start StepBackAndSteerTurnRecovery!!!********");
  ROS_INFO_NAMED ("top", "*****************************************************");

  std_msgs::Bool run_state;

  // when starting recovery, topic /run_state_ shifts to true
  run_state.data = true;
  recover_run_pub_.publish(run_state);

  cancel_ = false;

  int cnt = 0;
  const double stop_duaration = 1.0;
  while(ros::ok() && !cancel_)
  {
    cnt++;
    ROS_INFO_NAMED ("top", "==== %d th recovery trial ====", cnt);

    // Figure out how long we can safely run the behavior
    const geometry_msgs::Pose2D& initialPose = getCurrentLocalPose();

    // initial pose
    ROS_DEBUG_NAMED ("top", "initial pose (%.2f, %.2f, %.2f)", initialPose.x,
                     initialPose.y, initialPose.theta);
    ros::Rate r(controller_frequency_);

    // step back
    base_frame_twist_.linear.x = linear_vel_back_;
    ROS_INFO_NAMED ("top", "attempting step back");
    moveSpacifiedLength(base_frame_twist_, step_back_length_, BACKWARD);
    ROS_INFO_NAMED ("top", "complete step back");

    double final_diff = getCurrentDiff(initialPose);
    ROS_DEBUG_NAMED ("top", "final_diff = %.2f",final_diff);

    // stop
    for (double t=0; t<stop_duaration; t += 1/controller_frequency_) {
      cmd_vel_pub_.publish(TWIST_STOP);
      r.sleep();
    }

    int turn_dir = determineTurnDirection();
    int costmap_search_mode[CNT_TURN];

    double z;
    if(turn_dir == LEFT)
    {
      z = angular_speed_steer_;
      costmap_search_mode[FIRST_TURN] = FORWARD_LEFT;
      costmap_search_mode[SECOND_TURN] = FORWARD_RIGHT;
      ROS_INFO_NAMED ("top", "attempting to turn left at the 1st turn");
    }
    else
    {
      z = -1 * angular_speed_steer_;
      costmap_search_mode[FIRST_TURN] = FORWARD_RIGHT;
      costmap_search_mode[SECOND_TURN] = FORWARD_LEFT;
      ROS_INFO_NAMED ("top", "attemping to turn right at the 1st turn");
    }

    // clear way
    //-- first steering
    geometry_msgs::Twist twist;
    twist = TWIST_STOP;
    twist.linear.x = linear_vel_steer_;
    twist.angular.z = z;
    moveSpacifiedLength(twist, turn_angle_, (COSTMAP_SEARCH_MODE)costmap_search_mode[FIRST_TURN]);
    ROS_INFO_NAMED ("top", "complete the 1st turn");

    if(!only_single_steering_) {
      //-- go straight
      ROS_INFO_NAMED ("top", "attemping step forward");
      twist = TWIST_STOP;
      twist.linear.x = linear_vel_forward_;
      moveSpacifiedLength(twist, step_forward_length_, FORWARD);
      ROS_INFO_NAMED ("top", "complete step forward");

      //-- second steering
      ROS_INFO_NAMED ("top", "attempting second turn");
      twist = TWIST_STOP;
      twist.linear.x = linear_vel_steer_;
      twist.angular.z = -z;
      moveSpacifiedLength(twist, turn_angle_, (COSTMAP_SEARCH_MODE)costmap_search_mode[SECOND_TURN]);
      ROS_INFO_NAMED ("top", "complete second turn");
    }

    // stop
    for (double t=0; t<stop_duaration; t += 1/controller_frequency_) {
      cmd_vel_pub_.publish(TWIST_STOP);
      r.sleep();
    }

    // check trial times
    if(cnt == trial_times_)
    {
      ROS_INFO_NAMED ("top", "break after %d times recovery", cnt);
      break;
    }

    // check clearance forward
    const  geometry_msgs::Pose2D& current = getCurrentLocalPose();
    double max_angle = 0.1;
    double min_angle = -max_angle;
    double max_clearance = 0;
    twist.linear.x = 3.0;
    for(double angle = min_angle; angle < max_angle; angle += sim_angle_resolution_)
    {
      twist.angular.z = angle;
      geometry_msgs::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
      double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

      if(dist_to_obstacle > max_clearance)
        max_clearance = dist_to_obstacle;
    }

    if(max_clearance < 3.0)
    {
      ROS_INFO_NAMED ("top", "continue recovery because the robot couldn't get clearance");
      ROS_DEBUG_NAMED ("top", "continue at (%.2f, %.2f, %.2f) for max_clearance %.2f m",
                       twist.linear.x, twist.linear.y, twist.angular.z, max_clearance);
      continue;
    }
    else
    {
      ROS_INFO_NAMED ("top", "break recovery because the robot got clearance");
      ROS_DEBUG_NAMED ("top", "break at (%.2f, %.2f, %.2f) for max_clearance %.2f m",
                       twist.linear.x, twist.linear.y, twist.angular.z, max_clearance);
      break;
    }
  }

  // when finishing recovery, topic /run_state_ shifts to false
  run_state.data = false;
  recover_run_pub_.publish(run_state);

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "********Finish StepBackAndSteerTurnRecovery!!********");
  ROS_INFO_NAMED ("top", "*****************************************************");

  return 0;
}

bool StepBackAndSteerTurnRecovery::cancel() {
  cancel_ = true;
  return true;
}

} // namespace stepback_and_steerturn_recovery