/**
 *
 * @file teb_local_planner_ros_execution.h
 * @brief The trajectory planner ros executive class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_TEB_LOCAL_PLANNER_ROS_EXECUTION_H
#define CONTROLLER_EXECUTIVE_TEB_LOCAL_PLANNER_ROS_EXECUTION_H

#include <controller_executive/base_controller_execution.h>

namespace teb_local_planner {

 class TebLocalPlannerROSExecution : public controller::AbstractControllerExecution {

 public:

   TebLocalPlannerROSExecution();

  ~TebLocalPlannerROSExecution();
};

}

#endif //CONTROLLER_EXECUTIVE_TEB_LOCAL_PLANNER_ROS_EXECUTION_H
