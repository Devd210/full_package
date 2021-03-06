/**
 *
 * @file back_up_recovery.h
 * @brief back up recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_H
#define RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_H

#include <recovery_executive/base_recovery.h>
#include <mw_core/types.h>
#include <string>

namespace recovery
{
/**
 * @class BackU[Recovery
 * @brief A recovery behavior that backs up the robot to attempt to clear out space
 */
class BackUpRecovery : public recovery::AbstractRecovery
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  BackUpRecovery();

  /**
   * @brief  Initialization function for the BackUpRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   */
  void initialize(const std::string &name, const TFPtr &tf_listener_ptr) override;

  /**
   * @brief  Run the BackUpRecovery recovery behavior.
   */
  uint32_t runBehavior(std::string& message) override;

  bool cancel() override;

  /**
   * @brief  Destructor for the back up recovery behavior
   */
  ~BackUpRecovery();

private:
  bool initialized_;
  bool cancel_;
  double min_vel_x_;
  double max_vel_x_;
  double acc_lim_x_;
  double tolerance_;
  double frequency_;
  double distance_;

  std::string robot_base_frame_;
  std::string global_frame_;
};
};  // namespace recovery


#endif //RECOVERY_EXECUTIVE_BACK_UP_RECOVERY_H
