/**
 *
 * @file pure_pursuit_controller.cpp
 * @brief The pure pursuit controller class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/pure_pursuit_controller/pure_pursuit_controller.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(controller::PurePursuitController, controller::AbstractController);

namespace controller {

void PurePursuitController::reconfigureCB(controller_executive::PurePursuitControllerConfig &config, uint32_t level) {
  if (config.restore_defaults) {
    config = default_config_;
    //Avoid looping
    config.restore_defaults = false;
  }
  params_.min_lookahead = config.min_lookahead;
  params_.max_lookahead = config.max_lookahead;
  params_.closest_point_index_search = config.closest_point_index_search; 
  params_.max_acceleration = config.max_acceleration;
  params_.max_vel = config.max_vel;
  params_.min_vel = config.min_vel;
  params_.max_omega = config.max_omega;
  params_.max_radius = config.max_turn_radius;
  params_.min_radius = config.min_turn_radius;
  params_.max_omega_radius = config.max_radius_omega;
  params_.max_y_deviation = config.max_y_deviation;
  params_.max_path_dev = config.max_path_deviation;
  params_.max_theta_dev = config.max_theta_deviation;
  params_.transform_tolerance = config.transform_tolerance;
  holonomic_robot_ = config.holonomic_robot;
  drive_type_ = config.drive_type;
  on_the_spot_turn_allowed_ = config.on_the_spot_turn_allowed;
}

PurePursuitController::PurePursuitController() {

  std::string name = "pure_pursuit_controller";
  ros::NodeHandle nh("~");
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("min_lookahead", params_.min_lookahead, 0.2);
  private_nh.param("max_lookahead", params_.max_lookahead, 2.0);
  private_nh.param("closest_point_index_search", params_.closest_point_index_search, 10.0);
  private_nh.param("max_acceleration", params_.max_acceleration, 0.5);
  private_nh.param("max_vel", params_.max_vel, 0.2);
  private_nh.param("min_vel", params_.min_vel, 0.1);
  private_nh.param("max_omega", params_.max_omega, 0.5);
  private_nh.param("max_turn_radius", params_.max_radius, 0.2);
  private_nh.param("min_turn_radius", params_.min_radius, 0.1);
  private_nh.param("max_radius_omega", params_.max_omega_radius, 20.0);
  private_nh.param("max_y_deviation", params_.max_y_deviation, 0.05);
  private_nh.param("max_path_deviation", params_.max_path_dev, 1.0);
  private_nh.param("max_theta_deviation", params_.max_theta_dev, 0.5);
  private_nh.param("transform_tolerance", params_.transform_tolerance, 0.5);
  private_nh.param("holonomic_robot", holonomic_robot_, false);
  private_nh.param("drive_type", drive_type_, std::string("differential"));
  private_nh.param("on_the_spot_turn_allowed", on_the_spot_turn_allowed_, false);
  nh.param("robot_frame", params_.robot_base_frame, std::string("base_link"));
  nh.param("map_frame", params_.map_frame, std::string("map"));
  nh.param("tf_timeout", params_.tf_timeout, 1.0);

  pubLookaheadGoal_ = nh.advertise<geometry_msgs::PointStamped> ("/lookahead_goal", 5);
  is_goal_reached_ = false;

  plan_.clear();

  if (std::fabs(params_.theta_saturation_lim_max) <= std::numeric_limits<double>::epsilon()) {
    params_.theta_saturation_lim_max = 0.2;
    ROS_WARN("[Pure Pursuit Controller]: theta theta_saturation_lim_max_ is not set and being defaulted to %f",
             params_.theta_saturation_lim_max);
  }

  omega_positive_dir_ = true;

  if (params_.max_acceleration == 0) {
    throw PurePursuitControllerException(11);
  }

  ramp_distance_ = std::pow(params_.max_vel, 2) / (2 * params_.max_acceleration);

  dsrv_ = new dynamic_reconfigure::Server<controller_executive::PurePursuitControllerConfig>(private_nh);
  dynamic_reconfigure::Server<controller_executive::PurePursuitControllerConfig>::CallbackType
      cb = boost::bind(&PurePursuitController::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

PurePursuitController::~PurePursuitController() {

}

// ########################################
// ###   AbstractController functions   ###
// ########################################

unsigned int PurePursuitController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                            const geometry_msgs::TwistStamped &velocity,
                                                            geometry_msgs::TwistStamped &cmd_vel,
                                                            std::string &message) {

  if (plan_.empty()) {
    ROS_FATAL("[Pure Pursuit Controller]: No plan provided!");
    return 1;
  }

  current_velocity_ = velocity;
  current_pose_.x = pose.pose.position.x;
  current_pose_.y = pose.pose.position.y;
  current_pose_.heading = principleTheta(getYawFromPose(pose));

  if (compute()) {
    cmd_vel = cmd_vel_;
    // TODO: Check error codes
    return 0;
  }

  return 1;
}

bool PurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {

  geometry_msgs::PoseStamped pose;
  if (!mw_core::getRobotPose(*tf_listener_ptr_, params_.robot_base_frame, params_.map_frame,
                             ros::Duration(params_.tf_timeout), pose)) {
    ROS_ERROR("[Pure Pursuit Controller]: Failed to get robot pose");
    error_code_ = 4;
    return false;
  }

  plan_ = convertToPose2D(plan);

  current_pose_.x = pose.pose.position.x;
  current_pose_.y = pose.pose.position.y;
  current_pose_.heading = principleTheta(getYawFromPose(pose));

  for (auto &goal : plan_) {
    // if (calculateXYDeviation(current_pose_, goal) < 0.1) {
    if (calculateXYDeviation(current_pose_, goal) < params_.max_path_dev) {
      start_pose_ = goal;
      break;
    }
  }
  goal_pose_ = plan_[plan_.size() - 1];

  double initial_XY_dev = calculateXYDeviation(current_pose_, start_pose_);
  double initial_theta_dev = std::fabs(calculateThetaDeviation(current_pose_, start_pose_));

  bool start_sanity = false;

  if (initial_XY_dev < params_.max_path_dev) {
    if (initial_theta_dev < params_.max_theta_dev || holonomic_robot_ || drive_type_ == "differential") {
      start_sanity = true;
    } else {
      error_code_ = 1;
      ROS_ERROR("[Pure Pursuit Controller]: theta dev: %f, and max allowed: %f",
                initial_theta_dev,
                params_.max_theta_dev);
    }
  } else {
    error_code_ = 2;
    ROS_ERROR("[Pure Pursuit Controller]: Path dev: Path dev, and max allowed: %f %f",
              initial_XY_dev,
              params_.max_path_dev);
    ROS_DEBUG("[Pure Pursuit Controller]: Path Dev out of bounds-  current %f %f  start- %f %f",
              current_pose_.x, current_pose_.y,
              start_pose_.x, start_pose_.y);
  }

  if (!start_sanity) {
    throw PurePursuitControllerException(error_code_);
  }

  closest_pose_idx_ = -1;
  error_code_ = 0;

  return start_sanity;
}

bool PurePursuitController::isGoalReached(double dist_tolerance, double angle_tolerance) {
  if (is_goal_reached_)
    return true;

  double distance = calculateXYDeviation(current_pose_, goal_pose_);
  double angle = std::fabs(calculateThetaDeviation(current_pose_, goal_pose_));

  if (distance < dist_tolerance && angle < angle_tolerance) {
    is_goal_reached_ = true;
    return true;
  }

  return false;
}

bool PurePursuitController::cancel() {
  is_goal_reached_ = false;
  plan_.clear();
  return true;
}


// #############################################
// ###   Pure Pursuit Controller functions   ###
// #############################################

bool PurePursuitController::compute() {
  bool _path_dev = true;
  bool _orien_dev = true;
  bool _radius_sanity = true;

  // calculate the differnce between current and start, current and goal
  ROS_DEBUG("[Pure Pursuit Controller]: calculating the deviation");
  distance_from_start_ = calculateXYDeviation(current_pose_, start_pose_);
  distance_to_goal_ = calculateXYDeviation(current_pose_, goal_pose_);

  // get index of the closest point
  ROS_DEBUG("[Pure Pursuit Controller]: finding the closest pose");
  closest_pose_idx_ = findClosestPose();

  ROS_DEBUG("[Pure Pursuit Controller]: closest pose index is : %d", closest_pose_idx_);
  if (closest_pose_idx_ == -1) {
    ROS_ERROR("[Pure Pursuit Controller]: couldn't get closest pose for : %f %f %f",
              current_pose_.x,
              current_pose_.y,
              current_pose_.heading);
    resetCmdVel();
    error_code_ = 5;
    throw PurePursuitControllerException(error_code_);
  } else if (closest_pose_idx_ == plan_.size() - 1) {
    // we have reached the last point -> the goal is reached
    is_goal_reached_ = true;
    return true;
  }

  // get the radius of curvature based on the closest point and the point next to it
  ROS_DEBUG("[Pure Pursuit Controller]: radius of curvature of GP");
  double _curv_of_plan = getRadiusOfCurvatureOfGoalPoints();

  if (_curv_of_plan > params_.max_radius) {
    _curv_of_plan = params_.max_radius;
  }
  if (_curv_of_plan < params_.min_radius) {
    _curv_of_plan = params_.min_radius;
  }
  ROS_DEBUG("[Pure Pursuit Controller]: curvature which would be used %f", _curv_of_plan);

  // get the lookahead
  double lookahead_temp = getLookahead(_curv_of_plan);
  ROS_DEBUG("[Pure Pursuit Controller]: output of getLookahead is %f", lookahead_temp);

  mw_core::Point closest_pose = plan_[closest_pose_idx_];

  // calculate deviation in path
  ROS_DEBUG("[Pure Pursuit Controller]: calculate XY deviation");
  path_deviation_ = calculateXYDeviation(current_pose_, closest_pose);
  if (path_deviation_ > params_.max_path_dev) {
    ROS_DEBUG("[Pure Pursuit Controller]: path_deviation: %f max: %f", path_deviation_, params_.max_path_dev);
    error_code_ = 2;
    throw PurePursuitControllerException(error_code_);
  }

  // calculate deviation in heading
  ROS_DEBUG("[Pure Pursuit Controller]: calculate theta deviation");
  double orientation_deviation = calculateThetaDeviation(closest_pose, current_pose_);

  if (std::fabs(orientation_deviation) > params_.max_theta_dev) {
    if (drive_type_ == "differential") {
      if (on_the_spot_turn_allowed_) {
        cmd_vel_.header.stamp = ros::Time::now();
        cmd_vel_.twist.linear.x = 0;
        cmd_vel_.twist.linear.y = 0;
        cmd_vel_.twist.linear.z = 0;
        cmd_vel_.twist.angular.x = 0;
        cmd_vel_.twist.angular.y = 0;
        cmd_vel_.twist.angular.z = std::copysign(params_.max_omega, orientation_deviation);
        error_code_ = 0;
        ROS_DEBUG("[Pure Pursuit Controller]: the calculated velocities are v: %f w: %f",
                  cmd_vel_.twist.linear.x,
                  cmd_vel_.twist.angular.z);
        return true;
      } 
      else {
        cmd_vel_.header.stamp = ros::Time::now();
        cmd_vel_.twist.linear.x = params_.max_omega*params_.min_radius;
        cmd_vel_.twist.linear.y = 0;
        cmd_vel_.twist.linear.z = 0;
        cmd_vel_.twist.angular.x = 0;
        cmd_vel_.twist.angular.y = 0;
        cmd_vel_.twist.angular.z = std::copysign(params_.max_omega, orientation_deviation);
        error_code_ = 0;
        ROS_DEBUG("[Pure Pursuit Controller]: the calculated velocities are v: %f w: %f",
                  cmd_vel_.twist.linear.x,
                  cmd_vel_.twist.angular.z);
        return true;
      }     
    }
    error_code_ = 1;
    ROS_ERROR("[Pure Pursuit Controller]: current dev and max dev are %f, %f with ego and closest %f, %f",
              orientation_deviation, params_.max_theta_dev, current_pose_.heading, closest_pose.heading);
    throw PurePursuitControllerException(error_code_);
  }

  //publishing lookahead point for debugging
  double lookahead = improveLookahead(lookahead_temp);

  mw_core::Point target_pose = getTargetPose(lookahead);
  geometry_msgs::PointStamped lookahead_goal;
  lookahead_goal.header.frame_id = params_.map_frame;
  lookahead_goal.point.x = target_pose.x;   
  lookahead_goal.point.y = target_pose.y;
  lookahead_goal.point.z = 0;

  pubLookaheadGoal_.publish(lookahead_goal);

  ROS_DEBUG(
      "[Pure Pursuit Controller]: lookahead is %f and t_pose.x %f t_pose.y %f t_pose.theta %f e_pose.x %f e_pose.y %f e_pose.theta %f",
      lookahead,
      target_pose.x,
      target_pose.y,
      target_pose.heading,
      current_pose_.x,
      current_pose_.y,
      current_pose_.heading);

  double target_radius = getTargetRadius(current_pose_, target_pose);

  if (target_radius < params_.min_radius) {
    error_code_ = 6;
    throw PurePursuitControllerException(error_code_);
  }

  if (target_radius > params_.max_omega_radius) {
    target_radius = params_.max_omega_radius;
  }

  // get the target velocity based on ramping and interpolation  by target radius
  ROS_DEBUG("[Pure Pursuit Controller]: calculate velocity");
  double target_velocity = calculateVelocity(target_radius);

  if (target_velocity > params_.max_vel) {
    ROS_DEBUG("[Pure Pursuit Controller]: target_velocity %f, max_vel %f", target_velocity, params_.max_vel);
    error_code_ = 7;
    throw PurePursuitControllerException(error_code_);
  }

  // angular velocity
  ROS_DEBUG("[Pure Pursuit Controller]: calculate omega");
  double theta_delta = headingError(current_pose_, target_pose);
  double target_omega = calculateOmegaAlternate(theta_delta);

  if (target_omega > params_.max_omega) {
    error_code_ = 8;
    throw PurePursuitControllerException(error_code_);
  }

  // finally setting the command velocity
  if (!_path_dev || !_orien_dev || !_radius_sanity) {
    error_code_ = 9;
    throw PurePursuitControllerException(error_code_);
  } else {
    cmd_vel_.header.stamp = ros::Time::now();
    cmd_vel_.twist.linear.x = target_velocity;
    cmd_vel_.twist.linear.y = 0;
    cmd_vel_.twist.linear.z = 0;
    cmd_vel_.twist.angular.x = 0;
    cmd_vel_.twist.angular.y = 0;
    cmd_vel_.twist.angular.z = target_omega;
    error_code_ = 0;
    ROS_DEBUG("[Pure Pursuit Controller]: the calculated velocities are v: %f w: %f",
              cmd_vel_.twist.linear.x,
              cmd_vel_.twist.angular.z);

    return true;
  }
  return false;
}
void PurePursuitController::resetCmdVel() {
  cmd_vel_.header.stamp = ros::Time::now();
  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.linear.z = 0;
  cmd_vel_.twist.angular.x = 0;
  cmd_vel_.twist.angular.y = 0;
  cmd_vel_.twist.angular.z = 0;
  ROS_DEBUG("[Pure Pursuit Controller]: resetting the cmd vel to 0");
}

double PurePursuitController::improveLookahead(double lookahead) {
  if (path_deviation_ > params_.max_y_deviation) {
    return params_.min_lookahead;
  }
  if (params_.max_y_deviation == 0) {
    throw PurePursuitControllerException(12);
  }
  double ret = lookahead - ((lookahead - params_.min_lookahead) / params_.max_y_deviation) * path_deviation_;
  return ret;
}

int PurePursuitController::findClosestPose() {
  int _ret = -1;
  double _min_dist = std::numeric_limits<double>::max();
  bool _has_start = closest_pose_idx_ >= 0;

  if (!_has_start) {

    for (int i = 0; i < plan_.size(); i++) {
      double
          temp_dist = std::sqrt(std::pow(current_pose_.x - plan_[i].x, 2) + std::pow(current_pose_.y - plan_[i].y, 2));
      if (temp_dist < _min_dist) {
        _ret = i;
        _min_dist = temp_dist;
      }
    }

    if (_min_dist > params_.max_path_dev) {
      _ret = -1;
    }

    return _ret;

  } else {

    for (int i = closest_pose_idx_ - params_.closest_point_index_search; i < closest_pose_idx_ + params_.closest_point_index_search; i++) {
      double
          temp_dist = std::sqrt(std::pow(current_pose_.x - plan_[i].x, 2) + std::pow(current_pose_.y - plan_[i].y, 2));
      if (temp_dist < _min_dist) {
        _ret = i;
        _min_dist = temp_dist;
      }
    }

    if (_min_dist > params_.max_path_dev) {
      _ret = -1;
    }

    if (_ret > -1) {
      ROS_DEBUG("[Pure Pursuit Controller]: current_pose x: %f current_pose y: %f temp x: %f  temp y: %f",
                current_pose_.x,
                current_pose_.y,
                plan_[_ret].x,
                plan_[_ret].y);

      ROS_DEBUG("[Pure Pursuit Controller]: find closest pose returning: %d", _ret);
    }

    return _ret;
  }
}

mw_core::Point PurePursuitController::getTargetPose(const double &lookahead) {
  mw_core::Point ret;
  int ret_ind = 0;
  ROS_DEBUG("[Pure Pursuit Controller]: the size of the plan_ %d", int(plan_.size()));
  int i = 0;
  double dist = 0;
  for (i = closest_pose_idx_; i < plan_.size(); i++) {
    dist = calculateXYDeviation(current_pose_, plan_[i]);
    if (dist > lookahead) {
      ROS_DEBUG("[Pure Pursuit Controller]: found the point");
      ret_ind = i;
      ret = plan_[ret_ind];
      break;
    }
  }
  if (i == plan_.size() && ret_ind == 0) {
    ROS_DEBUG("[Pure Pursuit Controller]: target pose: reached the last segment, extrapolating the target pose");
    double remaining_diff = lookahead - dist;
    ret.x = plan_[i - 1].x + remaining_diff * std::cos(plan_[i - 1].heading);
    ret.y = plan_[i - 1].y + remaining_diff * std::sin(plan_[i - 1].heading);
    ret.heading = plan_[i - 1].heading;
  } else {
    ROS_DEBUG("[Pure Pursuit Controller]: closest_pose_idx_ %d lookahead %f and final index %d",
              closest_pose_idx_,
              lookahead,
              ret_ind);
  }
  return ret;
}

double PurePursuitController::headingError(const mw_core::Point current_pose,
                                           const mw_core::Point target_pose) {
  double ret1, ret2, d_x, d_y, theta_req;

  d_x = target_pose.x - current_pose.x;
  d_y = target_pose.y - current_pose.y;

  if (std::fabs(d_x) <= std::numeric_limits<double>::epsilon()) {
    if (target_pose.y < current_pose.y) {
      theta_req = -1 * M_PI_2;
    } else {
      theta_req = M_PI_2;
    }
  } else {
    theta_req = std::atan2(d_y, d_x);
  }

  ret1 = std::abs(current_pose.heading - theta_req);
  ret2 = 2 * M_PI - ret1;
  if (ret1 < ret2) {
    return ret1;
  } else {
    return ret2;
  }
}

double PurePursuitController::calculateOmegaAlternate(double theta_err) {
  double ret;

  if (theta_err > params_.theta_saturation_lim_max) {
    ret = params_.max_omega;
    ROS_DEBUG("[Pure Pursuit Controller]: omega calculated in step 1 %f", ret);
  } else {
    if (params_.theta_saturation_lim_max == 0) {
      throw PurePursuitControllerException(13);
    }
    ret = (params_.max_omega / (params_.theta_saturation_lim_max)) * theta_err;
    ROS_DEBUG("[Pure Pursuit Controller]: omega calculated in step 2 %f", ret);
  }

  if (!omega_positive_dir_) {
    ret = -ret;
    ROS_DEBUG("[Pure Pursuit Controller]: omega calculated in step 3 %f", ret);
  }

  double curr_omega = current_velocity_.twist.angular.z;

  if (fabs(ret) > (fabs(curr_omega) + MAX_OMEGA_DELTA)) {
    ret = (fabs(curr_omega) + MAX_OMEGA_DELTA)*(fabs(ret)/ret);
    ROS_DEBUG("[Pure Pursuit Controller]: omega calculated in step 4 %f", ret);
  }

  if (fabs(ret) < (fabs(curr_omega) - MAX_OMEGA_DELTA)) {
    ret = (fabs(curr_omega) - MAX_OMEGA_DELTA)*(fabs(ret)/ret);
    ROS_DEBUG("[Pure Pursuit Controller]: omega calculated in step 5 %f", ret);
  }
  if (fabs(ret)> params_.max_omega)
  {
    ret = params_.max_omega*(fabs(ret)/ret);
  }
  ROS_DEBUG("[Pure Pursuit Controller]: final omega calculated and returned %f", ret);
  return ret;
}

double PurePursuitController::calculateVelocity(double target_radius) {
  if (target_radius > params_.max_radius) {
    target_radius = params_.max_radius;
  }

  if (params_.max_radius == params_.min_radius) {
    throw PurePursuitControllerException(14);
  }

  if (params_.max_acceleration == 0) {
    throw PurePursuitControllerException(11);
  }

  double ret = ((params_.max_vel - params_.min_vel) / (params_.max_radius - params_.min_radius))
      * (target_radius - params_.min_radius) + params_.min_vel;

  ramp_distance_ = std::pow(ret, 2) / (2 * params_.max_acceleration);

  if (distance_from_start_ < ramp_distance_) {
    if (distance_from_start_ < std::numeric_limits<double>::epsilon()) {
      ret = params_.min_vel;
    } else {
      ret = std::sqrt(2 * params_.max_acceleration * distance_from_start_);
    }
  }

  if (distance_to_goal_ < ramp_distance_) {
    ret = std::sqrt(2 * params_.max_acceleration * distance_to_goal_);
  }

  double curr_vel =
      std::sqrt(std::pow(current_velocity_.twist.linear.x, 2) + std::pow(current_velocity_.twist.linear.y, 2));

  if (ret > curr_vel + MAX_VEL_DELTA) {
    ret = curr_vel + MAX_VEL_DELTA;
  }

  if (ret < curr_vel - MAX_VEL_DELTA) {
    ret = curr_vel - MAX_VEL_DELTA;
  }

  // final check for max vel
  if (ret > params_.max_vel)
  {
    ret = params_.max_vel;
  }
  return ret;
}

double PurePursuitController::calculateThetaDeviation(const mw_core::Point &pose1,
                                                      const mw_core::Point &pose2) {
  double ang_1, ang_2, r1, r2;
  ang_1 = principleTheta(pose1.heading);
  ang_2 = principleTheta(pose2.heading);

  // to keep it between [-pi, pi]
  r1 = principleTheta(ang_1 - ang_2);
  return r1;
}

double PurePursuitController::getLookahead(const double &_curv) {
  if (params_.max_radius == params_.min_radius) {
    throw PurePursuitControllerException(14);
  }
  double ret = ((params_.max_lookahead - params_.min_lookahead) / (params_.max_radius - params_.min_radius))
      * (_curv - params_.min_radius) + params_.min_lookahead;
  ROS_DEBUG("[Pure Pursuit Controller]: max_lookahead %f , min_lookahead %f",
            params_.max_lookahead,
            params_.min_lookahead);
  return ret;
}

double PurePursuitController::getRadiusOfCurvatureOfGoalPoints() {
  double _ret;
  if (closest_pose_idx_ < 0 || closest_pose_idx_ == plan_.size()) {
    _ret = -1;
    return _ret;
  }
  mw_core::Point pose1 = plan_[closest_pose_idx_];
  mw_core::Point
      pose2 = plan_[closest_pose_idx_ + (int) std::ceil(params_.max_lookahead / LOOKAHEAD_POINT_DISTANCE)];
  _ret = getRadiusOfCurvature(pose1, pose2);
  ROS_DEBUG("[Pure Pursuit Controller]: outpout of getCurvature %f", _ret);
  return _ret;
}

double PurePursuitController::getRadiusOfCurvature(const mw_core::Point pose1,
                                                   const mw_core::Point pose2) {
  ROS_DEBUG("[Pure Pursuit Controller]: curv calc point 1 x: %f y: %f theta: %f", pose1.x, pose1.y, pose1.heading);
  ROS_DEBUG("[Pure Pursuit Controller]: curv calc point 2 x: %f y: %f theta: %f", pose2.x, pose2.y, pose2.heading);

  //double espilon_slop_comparison = std::numeric_limits<double>::epsilon();
  double epsilon_slope_comparison = 0.1;

  double m_1, m_2;
  double c_1, c_2;
  double ang_1, ang_2;
  bool is_vert_1, is_vert_2;
  mw_core::Point cen;

  if (pose1.heading < 0) {
    ang_1 = M_PI + pose1.heading;
  } else {
    ang_1 = pose1.heading;
  }

  if (pose2.heading < 0) {
    ang_2 = M_PI + pose2.heading;
  } else {
    ang_2 = pose2.heading;
  }

  if (fabs(ang_1 - M_PI_2) < epsilon_slope_comparison) {
    is_vert_1 = true;
    m_1 = 1;
  } else {
    is_vert_1 = false;
    m_1 = std::tan(ang_1);
  }

  if (fabs(ang_2 - M_PI_2) < epsilon_slope_comparison) {
    is_vert_2 = true;
    m_2 = 1;
  } else {
    is_vert_2 = false;
    m_2 = std::tan(ang_2);
  }

  if (is_vert_1) {
    c_1 = -m_1 * pose1.x;
  } else {
    c_1 = pose1.y - m_1 * pose1.x;
  }

  if (is_vert_2) {
    c_2 = -m_2 * pose2.x;
  } else {
    c_2 = pose2.y - m_2 * pose2.x;
  }

  double ret;
  if (std::round(m_1 * 100) == std::round(m_2 * 100)) {
    ret = params_.max_omega_radius;
  } else {
    cen = getIntersection(is_vert_1, m_1, c_1, is_vert_2, m_2, c_2);

    ROS_DEBUG("[Pure Pursuit Controller]: for the radius of curv calc, center x: %f y: %f, point x: %f y: %f",
              cen.x,
              cen.y,
              pose1.x,
              pose1.y);

    geometry_msgs::PointStamped cen_m;
    geometry_msgs::PointStamped cen_r;

    cen_m.point.x = cen.x;
    cen_m.point.y = cen.y;

    while (!getPointRobot(cen_m, cen_r)) {
      error_code_ = 10;
      ROS_WARN("[Pure Pursuit Controller]: in loop trying to transform center of turn in robot frame");
    }

    if (cen_r.point.y < 0) {
      omega_positive_dir_ = false;
    }
    if (cen_m.point.y > 0) {
      omega_positive_dir_ = true;
    }
    ret = calculateXYDeviation(cen, pose1);
  }
  ROS_DEBUG("[Pure Pursuit Controller]: for the radius of curv calc, the xy deviation %f", ret);
  return ret;
}

double PurePursuitController::calculateXYDeviation(const mw_core::Point &pose1,
                                                   const mw_core::Point &pose2) {
  return std::sqrt(std::pow(pose1.x - pose2.x, 2) + std::pow(pose1.y - pose2.y, 2));
}

mw_core::Point PurePursuitController::getIntersection(bool is_vert_1,
                                                      double m_1,
                                                      double c_1,
                                                      bool is_vert_2,
                                                      double m_2,
                                                      double c_2) {
  double _x, _y;
  mw_core::Point ret;

  if (!is_vert_1 && !is_vert_2) {
    if (m_1 == m_2) {
      throw PurePursuitControllerException(15);
    }
    _x = (c_2 - c_1) / (m_1 - m_2);
    ret.x = _x;
    _y = m_1 * _x + c_1;
    ret.y = _y;
    return ret;
  }

  if (is_vert_1 && !is_vert_2) {
    _x = -c_1;
    _y = m_2 * _x + c_2;
    ret.x = _x;
    ret.y = _y;
    return ret;
  }

  if (!is_vert_1 && is_vert_2) {
    _x = -c_2;
    _y = m_1 * _x + c_1;
    ret.x = _x;
    ret.y = _y;
    return ret;
  }
}

std::vector<mw_core::Point> PurePursuitController::convertToPose2D(const std::vector<geometry_msgs::PoseStamped> plan) {
  std::vector<mw_core::Point> ret;
  for (auto &pose : plan) {
    mw_core::Point temp_ret;
    temp_ret.x = pose.pose.position.x;
    temp_ret.y = pose.pose.position.y;
    temp_ret.heading = getYawFromPose(pose);
    ret.push_back(temp_ret);
  }
  return ret;
}

bool PurePursuitController::getPointRobot(geometry_msgs::PointStamped &in, geometry_msgs::PointStamped &out) {
  in.header.frame_id = params_.map_frame;
  in.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();
  try {
    tf_listener_ptr_->transform(in, out, params_.robot_base_frame);
  }
  catch (tf2::LookupException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: No Transform available Error looking up centre pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: Extrapolation Error looking up centre pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::InvalidArgumentException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: invlalid arguments int the transform function: %s\n", ex.what());
    return false;
  }
  catch (tf2::TimeoutException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: %s\n", ex.what());
    return false;
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("[Pure Pursuit Controller]: %s\n", ex.what());
    return false;
  }

  // check if global_pose time stamp is within transform tolerance
  if (current_time.toSec() - out.header.stamp.toSec() > params_.transform_tolerance) {
    ROS_WARN("[Pure Pursuit Controller]: difference going out of transform tolerance");
    return false;
  }
  return true;
}

double PurePursuitController::getTargetRadius(const mw_core::Point pose1, const mw_core::Point pose2) {
  // note that the direction of motion in the robot frame is the x axis....
  // whereas y axis is the lateral direction
  double ret;
  geometry_msgs::PointStamped target;
  geometry_msgs::PointStamped target_robot;

  target.point.x = pose2.x;
  target.point.y = pose2.y;

  while (!getPointRobot(target, target_robot)) {
    error_code_ = 10;
    ROS_WARN("[Pure Pursuit Controller]: in loop trying to transform target in robot frame");
  }

  ROS_DEBUG("[Pure Pursuit Controller]: got target_pose in robot frame as x %f y %f ",
            target_robot.point.x,
            target_robot.point.y);

  omega_positive_dir_ = target_robot.point.y >= 0;

  // straight line, then assume it to be the straight line i.e. give it the max target radius.
  if (std::fabs(target_robot.point.y) < 0.01) {
    ret = params_.max_omega_radius;
  } else {
    // the derivation of the formula below is mention in the craig's paper about PP
    double d = ((target_robot.point.x * target_robot.point.x) - (target_robot.point.y * target_robot.point.y))
        / (2 * target_robot.point.y);
    ret = std::fabs(target_robot.point.y + d);
  }

  return ret;
}

void PurePursuitController::reset() {
  is_goal_reached_ = false;
  plan_.clear();
  omega_positive_dir_ = true;
  ramp_distance_ = std::pow(params_.max_vel, 2) / (2 * params_.max_acceleration);
}

}