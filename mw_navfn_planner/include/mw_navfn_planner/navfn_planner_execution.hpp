/**
 *
 * @file navfn_planner_execution.hpp
 * @brief The navfn planner.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef NAVFN_PLANNER__EXECUTION_HPP_
#define NAVFN_PLANNER__EXECUTION_HPP_

#include <global_planner/base_planner_execution.h>

namespace navfn_planner {

class NavfnPlannerExecution : public global_planner::AbstractPlannerExecution {

public:

  NavfnPlannerExecution();

  ~NavfnPlannerExecution();
};

}

#endif  // NAVFN_PLANNER__EXECUTION_HPP_
