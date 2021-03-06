/**
 *
 * @file base_recovery.h
 * @brief abstract recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_BASE_RECOVERY_H
#define RECOVERY_EXECUTIVE_BASE_RECOVERY_H

#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>

#include <mw_core/types.h>

namespace recovery {
/**
 * @class AbstractRecovery
 * @brief Provides an interface for recovery behaviors used in navigation.
 * All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
 */
class AbstractRecovery {

public:

  typedef boost::shared_ptr<::recovery::AbstractRecovery> Ptr;

  /**
   * @brief Initialize tf listener
   */
  virtual void initialize(const std::string &name,
                          const TFPtr &tf_listener_ptr) {
    name_ = name;
    tf_listener_ptr_ = tf_listener_ptr;
  };

  /**
   * @brief Runs the AbstractRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message) = 0;

  /**
   * @brief Virtual destructor for the interface
   */
  virtual ~AbstractRecovery(){}

  /**
   * @brief Requests the recovery behavior to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

protected:
  /**
   * @brief Constructor
   */
  AbstractRecovery(){};

  //! shared pointer to the shared tf listener
  TFPtr tf_listener_ptr_;

  std::string name_;
};

};  /* namespace recovery */

#endif  // RECOVERY_EXECUTIVE_BASE_RECOVERY_H
