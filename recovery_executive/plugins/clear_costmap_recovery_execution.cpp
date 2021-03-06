/**
 *
 * @file clear_costmap_recovery_execution.cpp
 * @brief The clear costmap recovery execution class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/plugins/clear_costmap_recovery_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(recovery::ClearCostmapRecoveryExecution, recovery::AbstractRecoveryExecution);

namespace recovery {

ClearCostmapRecoveryExecution::ClearCostmapRecoveryExecution() {

}

ClearCostmapRecoveryExecution::~ClearCostmapRecoveryExecution() {

}

}
