#include <boost/exception/diagnostic_information.hpp>

#include <recovery_executive/base_recovery_execution.h>

namespace recovery
{

AbstractRecoveryExecution::AbstractRecoveryExecution()
: AbstractExecution(),
  behavior_(nullptr),
  tf_listener_ptr_(nullptr),
  state_(INITIALIZED)
{
}

AbstractRecoveryExecution::~AbstractRecoveryExecution()
{
}

void AbstractRecoveryExecution::initialize(const std::string & name,
                                           const recovery::AbstractRecovery::Ptr & recovery_ptr,
                                           const TFPtr &tf_listener_ptr) {
  AbstractExecution::initialize(name);
  tf_listener_ptr_ = tf_listener_ptr;
  behavior_ = recovery_ptr;
  ROS_DEBUG("[Recovery Execution]: Abstract Recovery Execution initialized!");
}

void AbstractRecoveryExecution::setState(RecoveryState state)
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  state_ = state;
}

typename AbstractRecoveryExecution::RecoveryState AbstractRecoveryExecution::getState()
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return state_;
}

bool AbstractRecoveryExecution::cancel()
{
  cancel_ = true;
  // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
  if(!behavior_->cancel())
  {
    ROS_WARN_STREAM("[Recovery Execution]: Cancel recovery behavior \""
                    << name_ << "\" failed or is not supported by the plugin. "
                    << "Wait until the current recovery behavior finished!");
    return false;
  }
  return true;
}

void AbstractRecoveryExecution::postRun() {
  reset();
}

bool AbstractRecoveryExecution::reset() {
  setState(INITIALIZED);
  cancel_ = false;
  ROS_DEBUG("[Recovery Execution]: recovery execution was reset");
  return true;
}

bool AbstractRecoveryExecution::isPatienceExceeded()
{
  boost::lock_guard<boost::mutex> guard1(conf_mtx_);
  boost::lock_guard<boost::mutex> guard2(time_mtx_);
  ROS_DEBUG_STREAM("[Recovery Execution]: Patience: " << patience_ << ", start time: " << start_time_ << " now: " << ros::Time::now());
  return !patience_.isZero() && (ros::Time::now() - start_time_ > patience_);
}

void AbstractRecoveryExecution::run()
{
  cancel_ = false; // reset the canceled state

  time_mtx_.lock();
  start_time_ = ros::Time::now();
  time_mtx_.unlock();
  setState(RECOVERING);
  try
  {
    outcome_ = behavior_->runBehavior(message_);
    if (cancel_)
    {
      setState(CANCELED);
    }
    else
    {
      setState(RECOVERY_DONE);
    }
  }
  catch (boost::thread_interrupted &ex)
  {
    ROS_WARN_STREAM("[Recovery Execution]: Recovery \"" << name_ << "\" interrupted!");
    setState(STOPPED);
  }
  catch (...){
    ROS_FATAL_STREAM("[Recovery Execution]: Unknown error occurred in recovery behavior \"" << name_ << "\": " << boost::current_exception_diagnostic_information());
    setState(INTERNAL_ERROR);
  }
  condition_.notify_one();
}

} /* namespace recovery */
