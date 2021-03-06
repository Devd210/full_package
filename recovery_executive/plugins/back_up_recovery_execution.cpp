/**
 *
 * @file back_up_recovery_execution.cpp
 * @brief The back up recovery execution class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/plugins/back_up_recovery_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(recovery::BackUpRecoveryExecution, recovery::AbstractRecoveryExecution);

namespace recovery {

BackUpRecoveryExecution::BackUpRecoveryExecution() {

}

BackUpRecoveryExecution::~BackUpRecoveryExecution() {

}

}
