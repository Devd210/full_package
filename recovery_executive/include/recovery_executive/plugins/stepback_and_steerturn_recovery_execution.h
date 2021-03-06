/**
 *
 * @file stepback_and_steerturn_recovery_execution.h
 * @brief The stepback and steerturn recovery class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_EXECUTION_H
#define RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_EXECUTION_H

#include <recovery_executive/base_recovery_execution.h>

namespace recovery {

class StepBackAndSteerTurnRecoveryExecution : public recovery::AbstractRecoveryExecution {

 public:

  StepBackAndSteerTurnRecoveryExecution();

  ~StepBackAndSteerTurnRecoveryExecution();
};

}

#endif  // RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_EXECUTION_H
