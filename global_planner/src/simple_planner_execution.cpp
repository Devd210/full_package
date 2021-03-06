/**
 * @file simple_planner_execution.cpp
 * @brief simple planner execution class
 * @author Sourav Agrawal<sourav.agrawal@mowito.in>
 */

#include "global_planner/simple_planner_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::SimplePlannerExecution, global_planner::AbstractPlannerExecution);

namespace global_planner {

SimplePlannerExecution::SimplePlannerExecution() {

}

SimplePlannerExecution::~SimplePlannerExecution() {

}

}