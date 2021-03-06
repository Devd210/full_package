/**
 *
 * @file teb_local_planner_ros_execution.cpp
 * @brief The trajectory planner ros executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "mw_teb/teb_local_planner_ros_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROSExecution, controller::AbstractControllerExecution);

namespace teb_local_planner {

TebLocalPlannerROSExecution::TebLocalPlannerROSExecution() {

}

TebLocalPlannerROSExecution::~TebLocalPlannerROSExecution() {

}

}
