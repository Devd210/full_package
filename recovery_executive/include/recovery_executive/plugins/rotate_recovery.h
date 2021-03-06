/**
 *
 * @file rotate_recovery.h
 * @brief rotate recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_ROTATE_RECOVERY_H
#define RECOVERY_EXECUTIVE_ROTATE_RECOVERY_H

#include <recovery_executive/base_recovery.h>
#include <mw_core/types.h>
#include <string>

namespace recovery
{
/**
 * @class RotateRecovery
 * @brief A recovery behavior that rotates the robot in-place to attempt to clear out space
 */
class RotateRecovery : public recovery::AbstractRecovery
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  RotateRecovery();

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

  /**
   * @brief  Destructor for the rotate recovery behavior
   */
  ~RotateRecovery();

private:
  bool initialized_;
  bool cancel_;
  double min_rotational_vel_;
  double max_rotational_vel_;
  double acc_lim_th_;
  double tolerance_;
  double frequency_;
  double angle_;

  std::string robot_base_frame_;
  std::string global_frame_;
};
};  // namespace rotate_recovery

#endif //RECOVERY_EXECUTIVE_ROTATE_RECOVERY_H
