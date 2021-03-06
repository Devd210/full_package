/**
 * @file simple_planner_execution.h
 * @brief simple planner execution class
 * @author Sourav Agrawal<sourav.agrawal@mowito.in>
 */

#ifndef GLOBAL_PLANNER_SIMPLE_PLANNER_EXECUTION_H
#define GLOBAL_PLANNER_SIMPLE_PLANNER_EXECUTION_H

#include <global_planner/base_planner_execution.h>

namespace global_planner {

class SimplePlannerExecution : public global_planner::AbstractPlannerExecution {

 public:

  SimplePlannerExecution();

  ~SimplePlannerExecution();
};

}

#endif //GLOBAL_PLANNER_SIMPLE_PLANNER_EXECUTION_H
