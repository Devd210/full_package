/**
 *
 * @file clear_costmap_recovery.cpp
 * @brief clear costmap recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_H
#define RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_H

#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <recovery_executive/base_recovery.h>

namespace recovery {
/**
 * @class ClearCostmapRecovery
 * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
 */
class ClearCostmapRecovery : public recovery::AbstractRecovery {
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   * @param
   * @return
   */
  ClearCostmapRecovery();

  /**
   * @brief  Initialization function for the RotateRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   */
  void initialize(const std::string &name, const TFPtr &tf_listener_ptr) override;

  /**
   * @brief  Run the RotateRecovery recovery behavior.
   */
  uint32_t runBehavior(std::string& message) override;

  bool cancel() override;

private:

  ros::ServiceClient reset_costmap_srv_;
  bool initialized_;
};
};

#endif //RECOVERY_EXECUTIVE_CLEAR_COSTMAP_RECOVERY_H
