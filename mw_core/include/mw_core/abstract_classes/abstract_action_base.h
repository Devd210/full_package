/**
 * 
 * @file abstract_action_base.h
 * @brief Contains the base action class. All the actions inherit from this class
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 * 
 */

#pragma once

#include <map>
#include <actionlib/server/action_server.h>
#include <mw_core/robot_information.h>

namespace mw_core {

template<typename Action, typename Execution>
class AbstractActionBase {

 public:

  typedef boost::shared_ptr<AbstractActionBase> Ptr;

  typedef typename actionlib::ActionServer<Action>::GoalHandle GoalHandle;

  typedef boost::function<void(GoalHandle &goal_handle, Execution &execution)> RunMethod;

  typedef struct {
    typename Execution::Ptr execution;
    boost::thread *thread_ptr;
    GoalHandle goal_handle;
  } ConcurrencySlot;

  AbstractActionBase(
      const std::string &name,
      const mw_core::RobotInformation &robot_info,
      const RunMethod run_method) : name_(name), robot_info_(robot_info), run_(run_method) {}

  virtual void start(
      GoalHandle &goal_handle,
      typename Execution::Ptr execution_ptr) {
    uint8_t slot = goal_handle.getGoal()->concurrency_slot;
    if (goal_handle.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING) {
      goal_handle.setCanceled();
    } else {
      slot_map_mtx_.lock();
      typename std::map<uint8_t, ConcurrencySlot>::iterator slot_it =
          concurrency_slots_.find(slot);
      slot_map_mtx_.unlock();
      if (slot_it != concurrency_slots_.end()) {
        // if there is a plugin running on the same slot, cancel it
        slot_it->second.execution->cancel();
        if (slot_it->second.thread_ptr->joinable()) {
          slot_it->second.thread_ptr->join();
        }
      }
      boost::lock_guard<boost::mutex> guard(slot_map_mtx_);
      // fill concurrency slot with the new goal handle, execution, and working thread
      concurrency_slots_[slot].goal_handle = goal_handle;
      concurrency_slots_[slot].goal_handle.setAccepted();
      concurrency_slots_[slot].execution = execution_ptr;
      concurrency_slots_[slot].thread_ptr = threads_.create_thread(boost::bind(
          &AbstractActionBase::runAndCleanUp, this,
          boost::ref(concurrency_slots_[slot].goal_handle), execution_ptr));
    }
  }

  virtual void cancel(GoalHandle &goal_handle) {
    uint8_t slot = goal_handle.getGoal()->concurrency_slot;

    boost::lock_guard<boost::mutex> guard(slot_map_mtx_);
    typename std::map<uint8_t, ConcurrencySlot>::iterator slot_it = concurrency_slots_.find(slot);
    if (slot_it != concurrency_slots_.end()) {
      concurrency_slots_[slot].execution->cancel();
    }
  }

  virtual void runAndCleanUp(GoalHandle &goal_handle, typename Execution::Ptr execution_ptr) {
    uint8_t slot = goal_handle.getGoal()->concurrency_slot;
    ROS_DEBUG("Starting the action");
    //execution_ptr->preRun();
    run_(goal_handle, *execution_ptr);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Finished action \"" << name_ << "\" run method, waiting for execution thread to finish.");
    execution_ptr->join();
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Execution thread for action \"" << name_ << "\" stopped, cleaning up execution leftovers.");
    boost::lock_guard<boost::mutex> guard(slot_map_mtx_);
    ROS_DEBUG_STREAM_NAMED(name_, "Exiting run method with goal status "
        << (int) concurrency_slots_[slot].goal_handle.getGoalStatus().status
        << ": " << concurrency_slots_[slot].goal_handle.getGoalStatus().text);
    threads_.remove_thread(concurrency_slots_[slot].thread_ptr);
    delete concurrency_slots_[slot].thread_ptr;
    concurrency_slots_.erase(slot);
    execution_ptr->postRun();
  }

  virtual void cancelAll() {
    ROS_INFO_STREAM_NAMED(name_, "Cancel all goals for \"" << name_ << "\".");
    boost::lock_guard<boost::mutex> guard(slot_map_mtx_);
    typename std::map<uint8_t, ConcurrencySlot>::iterator iter;
    for (iter = concurrency_slots_.begin(); iter != concurrency_slots_.end(); ++iter) {
      iter->second.execution->cancel();
    }
    threads_.join_all();
  }

 protected:
  const std::string &name_;
  const mw_core::RobotInformation &robot_info_;

  RunMethod run_;
  boost::thread_group threads_;
  std::map<uint8_t, ConcurrencySlot> concurrency_slots_;

  boost::mutex slot_map_mtx_;
};

} // namespace mw_core
