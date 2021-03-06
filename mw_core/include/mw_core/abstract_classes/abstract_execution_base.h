/**
 * 
 * @file abstract_execution_base.h
 * @brief The base execution class. Every execution must inherit this class.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 * 
 */

#pragma once

#include <boost/thread.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/chrono/thread_clock.hpp>

#include <ros/ros.h>

namespace mw_core {
/**
 * @brief The execution base class. All the abstract execution derive from here.
*/
class AbstractExecution {
 public:

  /**
   * @brief This function is used to initialize the name
   */
  virtual void initialize(const std::string &name) {
    name_ = name;
  }

  /**
   * @brief This function is used to start the thread.
   */
  virtual bool start() {
    thread_ = boost::thread(&AbstractExecution::run, this);
    return true;
  }

  /**
   * @brief This function is used to interrupt the thread.
   */
  virtual void stop() {
    ROS_WARN_STREAM("Trying to stop the planning rigorously by interrupting the thread!");
    thread_.interrupt();
  }

  /**
   * @brief This function is used to cancel the current execution.
   */
  virtual bool cancel() = 0;

  /**
   * @brief This function is used to join the thread.
   */
  void join() {
    thread_.join();
  }

  /**
   * @brief This function is used to wait for the state update of the underlying execution.
   * @param duration The duration for which the mutex lock is kept
   */
  void waitForStateUpdate(boost::chrono::microseconds const &duration) {
    boost::mutex mutex;
    boost::unique_lock<boost::mutex> lock(mutex);
    condition_.wait_for(lock, duration);
  }

  /**
   * @brief Gets the current plugin execution outcome
   */
  unsigned int getOutcome() {
    return outcome_;
  }

  /**
   * @brief Gets the current plugin execution message
   */
  std::string getMessage() {
    return message_;
  }

  /**
   * @brief Returns the name of the corresponding plugin.
   */
  std::string getName() {
    return name_;
  }

  /**
   * @brief Optional implementation-specific setup function, called right before execution.
   */
  virtual void preRun() {};

  /**
   * @brief Optional implementation-specific setup function, called right after execution.
   */
  virtual void postRun() {};

 protected:

  /**
   * @brief This function is used in a loop to do the necessary task.
   */
  virtual void run() = 0;

  /**
   * @brief Constructor for the class
   * @param name The name of the execution
   */
  AbstractExecution() : outcome_(255),
                        cancel_(false) {}

  boost::condition_variable condition_; //condition variable to wake up control thread
  boost::thread thread_;                //the controlling thread object
  bool cancel_;                         //flag for canceling controlling
  unsigned int outcome_;                //the last received plugin execution outcome
  std::string message_;                 //the last received plugin execution message
  std::string name_;                    //the plugin name
};
} // namespace mw_core