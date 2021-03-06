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
 * @file navfn_planner.cpp
 * @brief The navfn planner.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */
#include "mw_navfn_planner/navfn_planner.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navfn_planner::NavfnPlanner, global_planner::AbstractPlanner);

namespace navfn_planner
{

NavfnPlanner::NavfnPlanner()
  : costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  ROS_INFO("[Navfn Planner]: Destroying plugin %s of type NavfnPlanner", name_.c_str());
  planner_.reset();
}

void NavfnPlanner::initialize(const std::string &name)
{
  name_ = name;

  ros::NodeHandle nodeHandle;
  ros::NodeHandle private_nh("~/" + name);

  costmap_.reset(new costmap_2d::CostmapHelper(nodeHandle, "global_costmap"));
  while (!costmap_->isReady()) {
    ROS_WARN("[Navfn Planner]: Waiting for initial global costmap data fetch...");
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  ROS_WARN("[Navfn Planner]: Initial global costmap data fetch successful!");

  global_frame_ = costmap_->getGlobalFrameID();

  // Initialize parameters
  // Declare this plugin's parameters
  private_nh.param("tolerance", tolerance_, 2.0);
  private_nh.param("use_astar", use_astar_, false);
  private_nh.param("allow_unknown", allow_unknown_, true);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
}

unsigned int NavfnPlanner::makePlan(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                                    double tolerance,
                                    std::vector<geometry_msgs::PoseStamped> &plan,
                                    double &cost,
                                    std::string &message)
{
  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  cost = 0.0;
  tolerance_ = tolerance;

  for (int i = 0; i < waypoints.size() - 1; ++i) {
    geometry_msgs::PoseStamped start = waypoints[i];
    geometry_msgs::PoseStamped goal = waypoints[i + 1];
    nav_msgs::Path path;

    if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
      ROS_WARN("[Navfn Planner]: %s: failed to create plan with "
               "tolerance %.2f.", name_.c_str(), tolerance_);
      return 10;
    }

    for (int j = 0; j < path.poses.size(); j++) {
      path.poses[j].header.stamp = path.header.stamp;
      path.poses[j].header.frame_id = path.header.frame_id;
      plan.push_back(path.poses[j]);
    }
  }

  if (plan.empty())
    return 10;

  return 0;
}

bool NavfnPlanner::isPlannerOutOfDate()
{
  return (!planner_ ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()));
}

bool NavfnPlanner::makePlan(
  const geometry_msgs::Pose &start,
  const geometry_msgs::Pose &goal, double tolerance,
  nav_msgs::Path &plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  ROS_DEBUG("[Navfn Planner]: Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
            start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    ROS_WARN("[Navfn Planner]: Cannot create a plan: the robot's start position is off the global"
             " costmap. Planning will always fail, are you sure"
             " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my)) {
    ROS_WARN("[Navfn Planner]: The goal sent to the planner is off the global costmap."
             " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::Pose p, best_pose;
  p = goal;

  bool found_legal = false;
  double best_sdist = std::numeric_limits<double>::max();

  p.position.y = goal.position.y - tolerance;

  while (p.position.y <= goal.position.y + tolerance) {
    p.position.x = goal.position.x - tolerance;
    while (p.position.x <= goal.position.x + tolerance) {
      double potential = getPointPotential(p.position);
      double sdist = squared_distance(p, goal);
      if (potential < POT_HIGH && sdist < best_sdist) {
        best_sdist = sdist;
        best_pose = p;
        found_legal = true;
      }
      p.position.x += resolution;
    }
    p.position.y += resolution;
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);
    } else {
      ROS_ERROR("[Navfn Planner]: Failed to create a plan from potential when a legal"
                " potential was found. This shouldn't happen.");
    }
  }

  double yaw, d_x_next, d_x_prev, d_y_next, d_y_prev;
  for (int idx = 0 ; idx < plan.poses.size() - 1; ++idx) {

  	if(idx == 0){
	    d_x_next = plan.poses[idx + 1].pose.position.x - plan.poses[idx].pose.position.x;
	    d_y_next = plan.poses[idx + 1].pose.position.y - plan.poses[idx].pose.position.y;

	    d_x_prev = d_x_next;
	    d_y_prev = d_y_next;
	  }
	  else{
	    d_x_next = plan.poses[idx + 1].pose.position.x - plan.poses[idx].pose.position.x;
	    d_y_next = plan.poses[idx + 1].pose.position.y - plan.poses[idx].pose.position.y;
	    
	    d_x_prev = plan.poses[idx].pose.position.x - plan.poses[idx - 1].pose.position.x;
	    d_y_prev = plan.poses[idx].pose.position.y - plan.poses[idx - 1].pose.position.y;
	  }

	  if (std::fabs(d_x_next) <= std::numeric_limits<double>::epsilon()) {
	    if (d_y_next < 0) {
	      yaw = -1 * M_PI_2;
	    } 
	    else {
	      yaw = M_PI_2;
	    }
	  } 
	  else {
	  	if (d_x_next < 0 && d_y_next > 0){
	  	  yaw = std::atan((d_y_next/d_x_next + d_y_prev/d_x_prev)/2.0) + M_PI;	
	  	}
	  	else if (d_x_next < 0 && d_y_next < 0){
	  	  yaw = std::atan((d_y_next/d_x_next + d_y_prev/d_x_prev)/2.0) - M_PI;	
	  	}
	  	else{
	  	  yaw = std::atan((d_y_next/d_x_next + d_y_prev/d_x_prev)/2.0);	
	  	}
	  }

	  plan.poses[idx].pose.orientation = mw_core::quaternionFromYaw(yaw);

  }
  return !plan.poses.empty();
}

void NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::Pose &goal,
  nav_msgs::Path &plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
        squared_distance(goal, second_to_last_pose.pose)) {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

bool NavfnPlanner::computePotential(const geometry_msgs::Point &world_point)
{
  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return false;
  }

  int map_start[2];
  map_start[0] = 0;
  map_start[1] = 0;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_start);
  planner_->setGoal(map_goal);

  if (use_astar_) {
    return planner_->calcNavFnAstar();
  }

  return planner_->calcNavFnDijkstra();
}

bool NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::Pose &goal,
  nav_msgs::Path &plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    ROS_WARN("[Navfn Planner]: The goal sent to the navfn planner is off the global costmap."
             " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  int path_len = planner_->calcPath(costmap_->getSizeInCellsX() * 4);
  if (path_len == 0) {
    ROS_DEBUG("[Navfn Planner]: No path found\n");
    return false;
  }

  ROS_DEBUG("[Navfn Planner]: Path found, %d steps\n", path_len);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = global_frame_;

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan.header.stamp;
    pose.header.frame_id = plan.header.frame_id;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double NavfnPlanner::getPointPotential(const geometry_msgs::Point &world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool NavfnPlanner::validPointPotential(const geometry_msgs::Point &world_point)
{
  return validPointPotential(world_point, tolerance_);
}

bool NavfnPlanner::validPointPotential(
  const geometry_msgs::Point &world_point, double tolerance)
{
  const double resolution = costmap_->getResolution();

  geometry_msgs::Point p = world_point;
  p.y = world_point.y - tolerance;

  while (p.y <= world_point.y + tolerance) {
    p.x = world_point.x - tolerance;
    while (p.x <= world_point.x + tolerance) {
      double potential = getPointPotential(p);
      if (potential < POT_HIGH) {
        return true;
      }
      p.x += resolution;
    }
    p.y += resolution;
  }

  return false;
}

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    ROS_ERROR("[Navfn Planner]: worldToMap failed: wx,wy: %f,%f, "
              "size_x,size_y: %d,%d", wx, wy,
              costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  ROS_ERROR("[Navfn Planner]: worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
            costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void NavfnPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

}  // namespace navfn_planner
