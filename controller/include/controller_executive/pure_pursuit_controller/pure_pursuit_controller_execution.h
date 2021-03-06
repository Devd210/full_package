/**
 *
 * @file pure_pursuit_controller_execution.h
 * @brief The pure pursuit controller executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_EXECUTION_H
#define CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_EXECUTION_H

#include "controller_executive/base_controller_execution.h"

namespace controller {

class PurePursuitControllerExecution : public AbstractControllerExecution {

 public:

  PurePursuitControllerExecution();

  ~PurePursuitControllerExecution();
};

}

#endif //CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_EXECUTION_H
