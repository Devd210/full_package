// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

/**
 *
 * @file navfn_planner.hpp
 * @brief The navfn planner.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef NAVFN_PLANNER__NAVFN_PLANNER_HPP_
#define NAVFN_PLANNER__NAVFN_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#include "mw_navfn_planner/navfn.hpp"
#include "global_planner/base_planner.h"
#include "mw_core/utility_functions.h"
#include "mw_core/types.h"
#include "costmap_2d/costmap_helper.h"
#include "costmap_2d/cost_values.h"

namespace navfn_planner
{

class NavfnPlanner
  : public global_planner::AbstractPlanner
{
public:
  NavfnPlanner();
  ~NavfnPlanner();

  void initialize(const std::string &name) override;

  unsigned int makePlan(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                        double tolerance,
                        std::vector<geometry_msgs::PoseStamped> &plan,
                        double &cost,
                        std::string &message) override;

protected:
  // Compute a plan given start and goal poses, provided in global world frame.
  bool makePlan(
    const geometry_msgs::Pose &start,
    const geometry_msgs::Pose &goal, double tolerance,
    nav_msgs::Path &plan);

  // Compute the navigation function given a seed point in the world to start from
  bool computePotential(const geometry_msgs::Point &world_point);

  // Compute a plan to a goal from a potential - must call computePotential first
  bool getPlanFromPotential(
    const geometry_msgs::Pose &goal,
    nav_msgs::Path &plan);

  // Remove artifacts at the end of the path - originated from planning on a discretized world
  void smoothApproachToGoal(
    const geometry_msgs::Pose &goal,
    nav_msgs::Path &plan);

  // Compute the potential, or navigation cost, at a given point in the world
  // - must call computePotential first
  double getPointPotential(const geometry_msgs::Point &world_point);

  // Check for a valid potential value at a given point in the world
  // - must call computePotential first
  // - currently unused
  bool validPointPotential(const geometry_msgs::Point &world_point);
  bool validPointPotential(const geometry_msgs::Point &world_point, double tolerance);

  // Compute the squared distance between two points
  inline double squared_distance(
    const geometry_msgs::Pose &p1,
    const geometry_msgs::Pose &p2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return dx * dx + dy * dy;
  }

  // Transform a point from world to map frame
  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my);

  // Transform a point from map to world frame
  void mapToWorld(double mx, double my, double &wx, double &wy);

  // Set the corresponding cell cost to be free space
  void clearRobotCell(unsigned int mx, unsigned int my);

  // Determine if a new planner object should be made
  bool isPlannerOutOfDate();

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<NavFn> planner_;

  // Global Costmap
  std::shared_ptr<costmap_2d::CostmapHelper> costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_;

  // Whether to use the astar planner or default dijkstras
  bool use_astar_;
};

}  // namespace navfn_planner

#endif  // NAVFN_PLANNER__NAVFN_PLANNER_HPP_
