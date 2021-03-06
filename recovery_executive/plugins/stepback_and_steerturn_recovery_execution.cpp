/**
 *
 * @file stepback_and_steerturn_recovery_execution.cpp
 * @brief The stepback and steerturn recovery execution class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/plugins/stepback_and_steerturn_recovery_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(recovery::StepBackAndSteerTurnRecoveryExecution, recovery::AbstractRecoveryExecution);

namespace recovery {

StepBackAndSteerTurnRecoveryExecution::StepBackAndSteerTurnRecoveryExecution() {

}

StepBackAndSteerTurnRecoveryExecution::~StepBackAndSteerTurnRecoveryExecution() {

}

}
