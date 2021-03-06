/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <controller_executive/trajectory_planner/trajectory_planner_ros.h>

#include <controller_executive/trajectory_planner/goal_functions.h>
#include <controller_executive/trajectory_planner/exception.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>
#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(trajectory_planner::TrajectoryPlannerROS, controller::AbstractController)

namespace trajectory_planner {

void TrajectoryPlannerROS::reconfigureCB(controller_executive::TrajectoryPlannerConfig &config, uint32_t level) {
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    //Avoid looping
    config.restore_defaults = false;
  }
  if (!setup_) {
    default_config_ = config;
    setup_ = true;
  }
  tc_->reconfigure(config);
  reached_goal_ = false;
}

TrajectoryPlannerROS::TrajectoryPlannerROS() :
    world_model_(nullptr),
    tc_(nullptr),
    costmap_helper_(nullptr),
    setup_(false),
    initialized_(false) {}

void TrajectoryPlannerROS::initialize(const std::string &name, const TFPtr &tf_listener_ptr) {

  if (!isInitialized()) {

    controller::AbstractController::initialize(name, tf_listener_ptr);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~/trajectory_planner");
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

    costmap_helper_.reset(new costmap_2d::CostmapHelper(nh, "local_costmap"));

    last_tf_idx_ = 0;
    rot_stopped_velocity_ = 1e-2;
    trans_stopped_velocity_ = 1e-2;
    double sim_time, sim_granularity, angular_sim_granularity;
    int vx_samples, vtheta_samples;
    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist,
        escape_reset_theta;
    bool holonomic_robot, dwa, simple_attractor, heading_scoring;
    double heading_scoring_timestep;
    double max_vel_x, min_vel_x;
    double backup_vel;
    double stop_time_buffer;
    std::string world_model_type;
    rotating_to_goal_ = false;

    while (!costmap_helper_->isReady()) {
      ROS_WARN("[Trajectory Planner]: Waiting for initial local costmap data fetch...");
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    ROS_INFO("[Trajectory Planner]: Initial local costmap data fetch successful!");

    global_frame_ = costmap_helper_->getGlobalFrameID();
    robot_base_frame_ = "base_link";

    private_nh.param("prune_plan", prune_plan_, true);

    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
    private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
    private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
    private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

    private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

    private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

    //Since I screwed up nicely in my documentation, I'm going to add errors
    //informing the user if they've set one of the wrong parameters
    if (private_nh.hasParam("acc_limit_x"))
      ROS_ERROR(
          "You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

    if (private_nh.hasParam("acc_limit_y"))
      ROS_ERROR(
          "You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

    if (private_nh.hasParam("acc_limit_th"))
      ROS_ERROR(
          "You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
      sim_period_ = 0.05;
    else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if (controller_frequency > 0)
        sim_period_ = 1.0 / controller_frequency;
      else {
        ROS_WARN(
            "[Trajectory Planner]: A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_DEBUG("[Trajectory Planner]: Sim period is set to %.2f", sim_period_);

    private_nh.param("sim_time", sim_time, 1.0);
    private_nh.param("sim_granularity", sim_granularity, 0.025);
    private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
    private_nh.param("vx_samples", vx_samples, 3);
    private_nh.param("vtheta_samples", vtheta_samples, 20);

    private_nh.param("path_distance_bias", pdist_scale, 0.6);
    private_nh.param("goal_distance_bias", gdist_scale, 0.8);
    private_nh.param("occdist_scale", occdist_scale, 0.01);

    bool meter_scoring;
    if (!private_nh.hasParam("meter_scoring")) {
      ROS_WARN(
          "[Trajectory Planner]: Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
    } else {
      private_nh.param("meter_scoring", meter_scoring, false);

      if (meter_scoring) {
        // if (!costmap_helper_->updateResolution())
        //   throw TrajectoryPlannerException(1);

        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_helper_->getResolution();
        gdist_scale *= resolution;
        pdist_scale *= resolution;
        occdist_scale *= resolution;
      } else {
        ROS_WARN(
            "[Trajectory Planner]: Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
      }
    }

    private_nh.param("heading_lookahead", heading_lookahead, 0.325);
    private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
    private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
    private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
    private_nh.param("holonomic_robot", holonomic_robot, true);
    private_nh.param("max_vel_x", max_vel_x, 0.5);
    private_nh.param("min_vel_x", min_vel_x, 0.1);

    double max_rotational_vel;
    private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
    max_vel_th_ = max_rotational_vel;
    min_vel_th_ = -1.0 * max_rotational_vel;

    private_nh.param("min_in_place_vel_theta", min_in_place_vel_th_, 0.4);
    reached_goal_ = false;
    backup_vel = -0.1;
    if (private_nh.getParam("backup_vel", backup_vel))
      ROS_WARN(
          "The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

    //if both backup_vel and escape_vel are set... we'll use escape_vel
    private_nh.getParam("escape_vel", backup_vel);

    if (backup_vel >= 0.0)
      ROS_WARN(
          "You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

    private_nh.param("world_model", world_model_type, std::string("costmap"));
    private_nh.param("dwa", dwa, true);
    private_nh.param("heading_scoring", heading_scoring, false);
    private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

    simple_attractor = false;

    ROS_ASSERT_MSG(world_model_type == "costmap",
                   "[Trajectory Planner]: At this time, only costmap world models are supported by this controller");
    world_model_ = new CostmapModel(costmap_helper_);
    std::vector<double> y_vels = loadYVels(private_nh);

    footprint_spec_ = costmap_helper_->getRobotFootprint();

    tc_ = new TrajectoryPlanner(*world_model_,
                                costmap_helper_,
                                footprint_spec_,
                                acc_lim_x_,
                                acc_lim_y_,
                                acc_lim_theta_,
                                sim_time,
                                sim_granularity,
                                vx_samples,
                                vtheta_samples,
                                pdist_scale,
                                gdist_scale,
                                occdist_scale,
                                heading_lookahead,
                                oscillation_reset_dist,
                                escape_reset_dist,
                                escape_reset_theta,
                                holonomic_robot,
                                max_vel_x,
                                min_vel_x,
                                max_vel_th_,
                                min_vel_th_,
                                min_in_place_vel_th_,
                                backup_vel,
                                dwa,
                                heading_scoring,
                                heading_scoring_timestep,
                                meter_scoring,
                                simple_attractor,
                                y_vels,
                                stop_time_buffer,
                                sim_period_,
                                angular_sim_granularity);

    map_viz_.initialize(name,
                        global_frame_,
                        boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
    initialized_ = true;

    dsrv_ = new dynamic_reconfigure::Server<controller_executive::TrajectoryPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<controller_executive::TrajectoryPlannerConfig>::CallbackType
        cb = boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

  } else {
    ROS_WARN("[Trajectory Planner]: This planner has already been initialized, doing nothing");
  }
}

std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node) {
  std::vector<double> y_vels;

  std::string y_vel_list;
  if (node.getParam("y_vels", y_vel_list)) {
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep("[], ");
    tokenizer tokens(y_vel_list, sep);

    for (tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++) {
      y_vels.push_back(atof((*i).c_str()));
    }
  } else {
    //if no values are passed in, we'll provide defaults
    y_vels.push_back(-0.3);
    y_vels.push_back(-0.1);
    y_vels.push_back(0.1);
    y_vels.push_back(0.3);
  }

  return y_vels;
}

TrajectoryPlannerROS::~TrajectoryPlannerROS() {
  //make sure to clean things up
  delete dsrv_;

  if (tc_ != NULL)
    delete tc_;

  if (world_model_ != NULL)
    delete world_model_;
}

bool TrajectoryPlannerROS::stopWithAccLimits(const geometry_msgs::PoseStamped &global_pose,
                                             const geometry_msgs::PoseStamped &robot_vel,
                                             geometry_msgs::Twist &cmd_vel) {
  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx =
      sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
  double vy =
      sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));

  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

  //we do want to check whether or not the command is valid
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
                                        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if (valid_cmd) {
    ROS_DEBUG("[Trajectory Planner]: Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool TrajectoryPlannerROS::rotateToGoal(const geometry_msgs::PoseStamped &global_pose,
                                        const geometry_msgs::PoseStamped &robot_vel,
                                        double goal_th,
                                        geometry_msgs::Twist &cmd_vel) {
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
                                                  std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
                                                                                                       std::min(-1.0
                                                                                                                    * min_in_place_vel_th_,
                                                                                                                ang_diff));

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
  double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

  v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff));

  v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

  // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
  v_theta_samp = v_theta_samp > 0.0
                 ? std::min(max_vel_th_, std::max(min_in_place_vel_th_, v_theta_samp))
                 : std::max(min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, v_theta_samp));

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x,
                                        global_pose.pose.position.y,
                                        yaw,
                                        robot_vel.pose.position.x,
                                        robot_vel.pose.position.y,
                                        vel_yaw,
                                        0.0,
                                        0.0,
                                        v_theta_samp);

  ROS_DEBUG_THROTTLE(1, "[Trajectory Planner]: Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d",
                     v_theta_samp,
                     valid_cmd);

  if (valid_cmd) {
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }

  cmd_vel.angular.z = 0.0;
  return false;

}

bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!isInitialized()) {
    ROS_ERROR(
        "[Trajectory Planner]: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  ROS_WARN("[Trajectory Planner]: new plan was set!");
  //reset the global plan
  global_plan_.clear();
  global_plan_ = plan;

  //when we get a new plan, we also want to clear any latch we may have on goal tolerances
  xy_tolerance_latch_ = false;
  //reset the at goal flag
  reached_goal_ = false;
  last_tf_idx_ = 0;
  return true;
}

void TrajectoryPlannerROS::reset() {
  global_plan_.clear();

  //when we get a new plan, we also want to clear any latch we may have on goal tolerances
  xy_tolerance_latch_ = false;

  //reset the at goal flag
  reached_goal_ = false;

  last_tf_idx_ = 0;
}

unsigned int TrajectoryPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                           const geometry_msgs::TwistStamped &velocity,
                                                           geometry_msgs::TwistStamped &cmd_vel,
                                                           std::string &message) {
  if (!isInitialized()) {
    ROS_ERROR(
        "[Trajectory Planner]: This planner has not been initialized, please call initialize() before using this planner");
    return 1;
  }
  // ROS_INFO(
  //       "[Trajectory Planner]: This planner initialized");
  std::vector<geometry_msgs::PoseStamped> local_plan;
  geometry_msgs::PoseStamped global_pose = pose;

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  //get the global plan in our frame
  if (!transformGlobalPlan(*tf_listener_ptr_,
                           global_plan_,
                           global_pose,
                           costmap_helper_,
                           global_frame_,
                           transformed_plan,
                           last_tf_idx_)) {
    ROS_WARN("[Trajectory Planner]: Could not transform the global plan to the frame of the controller");
    return 1;
  }

  //now we'll prune the plan based on the position of the robot
  // std::cout<<"[Trajectory Planner trajectory_planner_ros]: size of transformed plan is:"<<transformed_plan.size()<<std::endl;
  if (prune_plan_)
    prunePlan(global_pose, transformed_plan, global_plan_);

  geometry_msgs::PoseStamped drive_cmds;
  drive_cmds.header.frame_id = robot_base_frame_;

  geometry_msgs::PoseStamped robot_vel;
  robot_vel.header.frame_id = velocity.header.frame_id;
  robot_vel.pose.position.x = velocity.twist.linear.x;
  robot_vel.pose.position.y = velocity.twist.linear.y;
  robot_vel.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, velocity.twist.angular.z);
  tf2::convert(q, robot_vel.pose.orientation);
  robot_vel.header.stamp = ros::Time();
  // std::cout<<"[Trajectory Planner trajectory_planner_ros]: size of transformed plan is:"<<transformed_plan.size()<<std::endl;
  /* For timing uncomment
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  */

  //if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty()) {
    ROS_ERROR("[Trajectory Planner]: transformed plan was empty!");
    return 1;
  }

  const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();
  //we assume the global goal is the last point in the global plan
  const double goal_x = goal_point.pose.position.x;
  const double goal_y = goal_point.pose.position.y;

  const double yaw = tf2::getYaw(goal_point.pose.orientation);

  double goal_th = yaw;

  //check to see if we've reached the goal position
  if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_) {
      xy_tolerance_latch_ = true;
    }

    double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= yaw_goal_tolerance_) {
      //set the velocity command to zero
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      rotating_to_goal_ = false;
      xy_tolerance_latch_ = false;
      reached_goal_ = true;
    } else {
      //we need to call the next two lines to make sure that the trajectory
      //planner updates its path distance and goal distance grids
      tc_->updatePlan(transformed_plan);
      // std::cout<<"[Trajectory Planner trajectory_planner_ros]: size of transformed plan is:"<<transformed_plan.size()<<std::endl;
      Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
      map_viz_.publishCostCloud(costmap_helper_);

      //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
      if (!rotating_to_goal_
          && !trajectory_planner::stopped(velocity.twist, rot_stopped_velocity_, trans_stopped_velocity_)) {
        if (!stopWithAccLimits(global_pose, robot_vel, cmd_vel.twist)) {
          return 1;
        }
      }
        //if we're stopped... then we want to rotate to goal
      else {
        //set this so that we know its OK to be moving
        rotating_to_goal_ = true;
        if (!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel.twist)) {
          return 1;
        }
      }
    }

    //publish an empty plan because we've reached our goal position
    publishPlan(transformed_plan, g_plan_pub_);
    // std::cout<<"[Trajectory Planner trajectory_planner_ros]: size of transformed plan is:"<<transformed_plan.size()<<std::endl;
    publishPlan(local_plan, l_plan_pub_);

    //we don't actually want to run the controller when we're just rotating to goal
    return 0;
  }

  tc_->updatePlan(transformed_plan);

  //compute what trajectory to drive along
  Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

  map_viz_.publishCostCloud(costmap_helper_);
  /* For timing uncomment
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  ROS_DEBUG("[Trajectory Planner]: Cycle time: %.9f", t_diff);
  */

  //pass along drive commands
  cmd_vel.twist.linear.x = drive_cmds.pose.position.x;
  cmd_vel.twist.linear.y = drive_cmds.pose.position.y;
  cmd_vel.twist.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

  //if we cannot move... tell someone
  if (path.cost_ < 0) {
    ROS_DEBUG(
        "[Trajectory Planner]: The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
    local_plan.clear();
    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return 1;
  }

  ROS_DEBUG("[Trajectory Planner]: A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
            cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);

  // Fill out the local plan
  for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
    double p_x, p_y, p_th;
    path.getPoint(i, p_x, p_y, p_th);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = p_x;
    pose.pose.position.y = p_y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p_th);
    tf2::convert(q, pose.pose.orientation);
    local_plan.push_back(pose);
  }

  //publish information to the visualizer
  publishPlan(transformed_plan, g_plan_pub_);
  publishPlan(local_plan, l_plan_pub_);
  return 0;
}

bool TrajectoryPlannerROS::isGoalReached(double dist_tolerance, double angle_tolerance) {
  if (!isInitialized()) {
    ROS_ERROR(
        "[Trajectory Planner]: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  //return flag set in controller
  return reached_goal_;
}

bool TrajectoryPlannerROS::cancel() {
  global_plan_.clear();
  reached_goal_ = false;
  return true;
}

};
