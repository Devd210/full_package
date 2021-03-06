/**
 * 
 * @file base_planner.h
 * @brief The base planner class. Every planner must inherit this class.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *  
 */

#pragma once

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace global_planner
{
/**
 * @class AbstractPlanner
 * @brief Provides an interface for global planners.All global planners written to work as executive plugins must adhere to this interface.
 */
class AbstractPlanner
{
public:
  typedef boost::shared_ptr<::global_planner::AbstractPlanner> Ptr;
  /**
   * @brief Virtual destructor for the AbstractPlanner class
   */
  virtual ~AbstractPlanner() {}
  /**
   * @brief Given a set of waypoints compute a plan
   * @param waypoints The waypoints to be used
   * @param tolerance The tolerance for goal
   * @param plan The computed plan is stored here
   * @param cost The total cost of the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result
   */
  virtual unsigned int makePlan(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                                double tolerance,
                                std::vector<geometry_msgs::PoseStamped> &plan,
                                double &cost,
                                std::string &message) = 0;
  /**
   * @brief Initialization function of the planner(only called by planners implementing costmaps)
   * @param name The name of the planner
   */
  virtual void initialize(const std::string &name) {
    name_ = name;
  }

protected:
  /**
   * @brief Protected constructor to make the class abstract
   */
  AbstractPlanner(){};

  std::string name_;
};
} // namespace global_planner