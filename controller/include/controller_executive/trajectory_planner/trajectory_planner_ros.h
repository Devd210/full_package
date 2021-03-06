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
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>

#include <controller_executive/base_controller.h>
#include <costmap_2d/costmap_helper.h>

#include <controller_executive/trajectory_planner/world_model.h>
#include <controller_executive/trajectory_planner/costmap_model.h>
#include <controller_executive/trajectory_planner/trajectory_planner.h>
#include <controller_executive/trajectory_planner/map_grid_visualizer.h>
#include <controller_executive/trajectory_planner/planar_laser_scan.h>

#include <controller_executive/TrajectoryPlannerConfig.h>

#include <costmap_2d/costmap_helper.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <mw_core/types.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <dynamic_reconfigure/server.h>

namespace trajectory_planner {
/**
 * @class TrajectoryPlannerROS
 * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
 */
class TrajectoryPlannerROS : public controller::AbstractController {
 public:
  /**
   * @brief  Default constructor for the ros wrapper
   */
  TrajectoryPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(const std::string &name, const TFPtr &tf_listener_ptr);

  /**
   * @brief  Destructor for the wrapper
   */
  ~TrajectoryPlannerROS();

  // ###################################
  // #  Abstract Controller Functions  #
  // ###################################

  unsigned int computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                       const geometry_msgs::TwistStamped &velocity,
                                       geometry_msgs::TwistStamped &cmd_vel,
                                       std::string &message) override;

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override;

  void reset() override;

  // ##################################
  // #  Trajectory Planner Functions  #
  // ##################################

  bool isInitialized() {
    return initialized_;
  }

  /** @brief Return the inner TrajectoryPlanner object.  Only valid after initialize(). */
  TrajectoryPlanner *getPlanner() const { 
    return tc_; 
  }

 private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(controller_executive::TrajectoryPlannerConfig &config, uint32_t level);

  /**
   * @brief Once a goal position is reached... rotate to the goal orientation
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  goal_th The desired th value for the goal
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool rotateToGoal(const geometry_msgs::PoseStamped &global_pose,
                    const geometry_msgs::PoseStamped &robot_vel,
                    double goal_th,
                    geometry_msgs::Twist &cmd_vel);

  /**
   * @brief Stop the robot taking into account acceleration limits
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool stopWithAccLimits(const geometry_msgs::PoseStamped &global_pose,
                         const geometry_msgs::PoseStamped &robot_vel,
                         geometry_msgs::Twist &cmd_vel);

  std::vector<double> loadYVels(ros::NodeHandle node);

  double sign(double x) {
    return x < 0.0 ? -1.0 : 1.0;
  }

  WorldModel *world_model_; ///< @brief The world model that the controller will use
  TrajectoryPlanner *tc_; ///< @brief The trajectory controller

  costmap_2d::CostmapHelper::Ptr costmap_helper_; ///< @brief The ROS wrapper for the costmap the controller will use
  
  MapGridVisualizer
      map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
  
  std::string global_frame_; ///< @brief The frame in which the controller will run
  std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
  double rot_stopped_velocity_, trans_stopped_velocity_;
  double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  bool prune_plan_;

  double max_vel_th_, min_vel_th_;
  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double sim_period_;
  bool rotating_to_goal_;
  bool reached_goal_;
  bool latch_xy_goal_tolerance_, xy_tolerance_latch_;

  ros::Publisher g_plan_pub_, l_plan_pub_;

  dynamic_reconfigure::Server<controller_executive::TrajectoryPlannerConfig> *dsrv_;
  controller_executive::TrajectoryPlannerConfig default_config_;
  bool setup_;

  bool initialized_;

  unsigned int last_tf_idx_;
  
  std::vector<geometry_msgs::Point> footprint_spec_;
};
};
#endif
