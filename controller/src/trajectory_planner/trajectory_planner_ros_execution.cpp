/**
 *
 * @file trajectory_planner_ros_execution.cpp
 * @brief The trajectory planner ros executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/trajectory_planner/trajectory_planner_ros_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(trajectory_planner::TrajectoryPlannerROSExecution, controller::AbstractControllerExecution);

namespace trajectory_planner {

TrajectoryPlannerROSExecution::TrajectoryPlannerROSExecution() {

}

TrajectoryPlannerROSExecution::~TrajectoryPlannerROSExecution() {

}

}