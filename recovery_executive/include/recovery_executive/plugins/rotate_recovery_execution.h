/**
 *
 * @file rotate_recovery_execution.h
 * @brief The rotate recovery class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_ROTATE_RECOVERY_EXECUTION_H
#define RECOVERY_EXECUTIVE_ROTATE_RECOVERY_EXECUTION_H

#include <recovery_executive/base_recovery_execution.h>

namespace recovery {

class RotateRecoveryExecution : public recovery::AbstractRecoveryExecution {

 public:

  RotateRecoveryExecution();

  ~RotateRecoveryExecution();
};

}

#endif  // RECOVERY_EXECUTIVE_ROTATE_RECOVERY_EXECUTION_H
