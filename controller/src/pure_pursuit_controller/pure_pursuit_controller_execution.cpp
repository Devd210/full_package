/**
 *
 * @file pure_pursuit_controller_execution.cpp
 * @brief The pure pursuit controller executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/pure_pursuit_controller/pure_pursuit_controller_execution.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(controller::PurePursuitControllerExecution, controller::AbstractControllerExecution);

namespace controller {

PurePursuitControllerExecution::PurePursuitControllerExecution() {

}

PurePursuitControllerExecution::~PurePursuitControllerExecution() {

}

}