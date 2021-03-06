/**
 *
 * @file rotate_recovery_execution.cpp
 * @brief The rotate recovery execution class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/plugins/rotate_recovery_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(recovery::RotateRecoveryExecution, recovery::AbstractRecoveryExecution);

namespace recovery {

RotateRecoveryExecution::RotateRecoveryExecution() {

}

RotateRecoveryExecution::~RotateRecoveryExecution() {

}

}
