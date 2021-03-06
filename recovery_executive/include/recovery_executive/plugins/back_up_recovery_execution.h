/**
 *
 * @file back_up_recovery_execution.h
 * @brief The back up recovery class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_EXECUTION_H
#define RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_EXECUTION_H

#include <recovery_executive/base_recovery_execution.h>

namespace recovery {

class BackUpRecoveryExecution : public recovery::AbstractRecoveryExecution {

 public:

  BackUpRecoveryExecution();

  ~BackUpRecoveryExecution();
};

}

#endif  // RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_EXECUTION_H
