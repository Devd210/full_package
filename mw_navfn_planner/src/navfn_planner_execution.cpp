/**
 *
 * @file navfn_planner_execution.cpp
 * @brief The navfn planner.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "mw_navfn_planner/navfn_planner_execution.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navfn_planner::NavfnPlannerExecution, global_planner::AbstractPlannerExecution);

namespace navfn_planner {

NavfnPlannerExecution::NavfnPlannerExecution() {

}

NavfnPlannerExecution::~NavfnPlannerExecution() {

}

}

