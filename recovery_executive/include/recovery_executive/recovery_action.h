/**
 *
 * @file recovery_action.h
 * @brief recovery action class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_RECOVERY_ACTION_H
#define RECOVERY_EXECUTIVE_RECOVERY_ACTION_H

#include "mw_core/abstract_classes/abstract_action_base.h"
#include "recovery_executive/base_recovery_execution.h"
#include "mw_core/robot_information.h"
#include <actionlib/server/action_server.h>
#include <mw_msgs/RecoveryAction.h>
#include <boost/thread/condition_variable.hpp>

namespace recovery {

class RecoveryAction : public mw_core::AbstractActionBase<mw_msgs::RecoveryAction, AbstractRecoveryExecution>
{
public:

  typedef boost::shared_ptr<RecoveryAction> Ptr;

  RecoveryAction(const std::string& name, const mw_core::RobotInformation &robot_info);

  void run(GoalHandle &goal_handle, AbstractRecoveryExecution &execution);

};

} /* recovery */

#endif //RECOVERY_EXECUTIVE_RECOVERY_ACTION_H
