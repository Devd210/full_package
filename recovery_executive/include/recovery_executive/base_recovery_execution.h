/**
 *
 * @file base_recovery_execution.h
 * @brief abstract recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_BASE_RECOVERY_EXECUTION_H
#define RECOVERY_EXECUTIVE_BASE_RECOVERY_EXECUTION_H

#include <map>
#include <string>
#include <stdint.h>
#include <vector>

#include <mw_core/utility_functions.h>
#include <mw_core/abstract_classes/abstract_execution_base.h>

#include <recovery_executive/base_recovery.h>

namespace recovery {
/**
 * @brief The AbstractRecoveryExecution class loads and binds the recovery behavior plugin. It contains a thread
 *        running the plugin, executing the recovery behavior. An internal state is saved and will be pulled by the
 *        server, which controls the recovery behavior execution. Due to a state change it wakes up all threads
 *        connected to the condition variable.
 *
 */
class AbstractRecoveryExecution : public mw_core::AbstractExecution
{
public:

  typedef boost::shared_ptr<AbstractRecoveryExecution> Ptr;

  /**
   * @brief internal state.
   */
  enum RecoveryState
  {
    INITIALIZED,   ///< The recovery execution has been initialized.
    STARTED,       ///< The recovery execution thread has been started.
    RECOVERING,    ///< The recovery behavior plugin is running.
    WRONG_NAME,    ///< The given name could not be associated with a load behavior.
    RECOVERY_DONE, ///< The recovery behavior execution is done.
    CANCELED,      ///< The recovery execution was canceled.
    STOPPED,       ///< The recovery execution has been stopped.
    INTERNAL_ERROR ///< An internal error occurred.
  };

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param tf_listener_ptr Shared pointer to a common tf listener
   */
  AbstractRecoveryExecution();

  /**
   * @brief Destructor
   */
  virtual ~AbstractRecoveryExecution();

  /**
   * @brief initialize
   */
  virtual void initialize(const std::string & name,
                          const recovery::AbstractRecovery::Ptr & recovery_ptr,
                          const TFPtr & tf_listener_ptr);

  /**
   * @brief Checks whether the patience was exceeded.
   * @return true, if the patience duration was exceeded.
   */
  bool isPatienceExceeded();

  /**
   * @brief Cancel the planner execution. This calls the cancel method of the planner plugin. This could be useful if the
   * computation takes too much time.
   * @return true, if the planner plugin tries / tried to cancel the planning step.
   */
  virtual bool cancel();

//  virtual void postRun();

  bool reset();

  virtual void postRun();
  
  /**
   * @brief Returns the current state, thread-safe communication
   * @return current internal state
   */
  AbstractRecoveryExecution::RecoveryState getState();

protected:

  /**
   * @brief Main execution method which will be executed by the recovery execution thread_.
   */
  virtual void run();

  //! the current loaded recovery behavior
  recovery::AbstractRecovery::Ptr behavior_;

  //! shared pointer to common TransformListener
  TFPtr tf_listener_ptr_;

private:

  /**
   * @brief Sets the current internal state. This method is thread communication safe
   * @param state The state to set.
   */
  void setState(RecoveryState state);

  //! mutex to handle safe thread communication for the current state
  boost::mutex state_mtx_;

  //! dynamic reconfigure and start time mutexes to mutually exclude read/write configuration
  boost::mutex conf_mtx_;
  boost::mutex time_mtx_;

  //! recovery behavior allowed time
  ros::Duration patience_;

  //! recovery behavior start time
  ros::Time start_time_;

  //! current internal state
  RecoveryState state_;
};

} /* namespace recovery */

#endif  // RECOVERY_EXECUTIVE_BASE_RECOVERY_EXECUTION_H
