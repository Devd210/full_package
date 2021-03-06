/**
 *
 * @file clear_costmap_recovery_execution.h
 * @brief The clear costmap recovery class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_EXECUTION_H
#define RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_EXECUTION_H

#include <recovery_executive/base_recovery_execution.h>

namespace recovery {

class ClearCostmapRecoveryExecution : public recovery::AbstractRecoveryExecution {

 public:

  ClearCostmapRecoveryExecution();

  ~ClearCostmapRecoveryExecution();
};

}

#endif  // RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_EXECUTION_H
