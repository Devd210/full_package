/**
 *
 * @file trajectory_planner_ros_execution.h
 * @brief The trajectory planner ros executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_TRAJECTORY_PLANNER_ROS_EXECUTION_H
#define CONTROLLER_EXECUTIVE_TRAJECTORY_PLANNER_ROS_EXECUTION_H

#include <controller_executive/base_controller_execution.h>

namespace trajectory_planner {

 class TrajectoryPlannerROSExecution : public controller::AbstractControllerExecution {

 public:

   TrajectoryPlannerROSExecution();

  ~TrajectoryPlannerROSExecution();
};

}

#endif //CONTROLLER_EXECUTIVE_TRAJECTORY_PLANNER_ROS_EXECUTION_H
