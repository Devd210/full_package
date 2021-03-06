#include <chief_executive/chief_executive.h>

namespace executive {

ChiefExecutiveClass::ChiefExecutiveClass() {
  auto_manual_ = false;
  start_ = false;
  emergency_stop_ = false;
  error_code_ = 0;

  default_factory_ = "base_fac";   //to be changed later when introducing multiple factories
  error_code_ = 0;
  robot_status_.chief_executive_mode = mw_msgs::ChiefExecutiveMode::WAITING_FOR_AUTO_MODE;  //either use robot_upstart package or launch the nodes that should run in this mode

  ros::NodeHandle private_nh("~");

  get_duty_ = private_nh.advertiseService("get_current_duty", &ChiefExecutiveClass::getDuty, this);
  set_current_duty_ = private_nh.advertiseService("set_current_duty", &ChiefExecutiveClass::setCurrentDuty, this);
  set_default_duty_ = private_nh.advertiseService("set_default_duty", &ChiefExecutiveClass::setDefaultDuty, this);
  set_ceo_mode_ = private_nh.advertiseService("set_chief_executive_mode", &ChiefExecutiveClass::setCEOMode, this);
  pause_duty_ = private_nh.advertiseService("pause_duty", &ChiefExecutiveClass::pauseDuty, this);

  execute_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ChiefExecutiveClass::executeCB, this, _1));

  set_route_client_ = nh_.serviceClient<mw_msgs::SetRoute>("mission_executive/set_route");
  route_status_client_ = nh_.serviceClient<mw_msgs::GetRouteStatus>("mission_executive/get_route_status");

  set_task_client_ = nh_.serviceClient<mw_msgs::SetTask>("task_executive/set_task");
  task_status_client_ = nh_.serviceClient<mw_msgs::GetTaskStatus>("task_executive/get_task_status");

  status_pub_ = nh_.advertise<mw_msgs::RobotStatus>("/robot_status", 10);

  //dummies
  battery_status_ = 90;
  robot_error_code_ = 0;

  auto_pause_ = false;

  ROS_INFO("[CEO]: chief executive is running!");
}

ChiefExecutiveClass::~ChiefExecutiveClass() {

}

bool ChiefExecutiveClass::pauseDuty(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
  if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
    auto_pause_ = req.data;
    resp.success = (unsigned char) true;
    return true;
  } else {
    resp.success = (unsigned char) false;
    resp.message = "robot not in auto mode";
    return false;
  }
}

bool ChiefExecutiveClass::getDuty(mw_msgs::GetDuty::Request &req, mw_msgs::GetDuty::Response &resp) {
  // Return current duty if request has ID = 0
  // Otherwise, return the duty corresponding to the requested ID
  // For this case, we will need to maintain to list of samples (to be done)
  if (req.id == 0) {
    ROS_WARN("[CEO]: sending current duty");
    resp.duty = current_duty_.duty;
    return true;
  }
  ROS_WARN("[CEO]: invalid duty ID requested");
  return false;
}

bool ChiefExecutiveClass::setDefaultDuty(mw_msgs::SetDuty::Request &req, mw_msgs::SetDuty::Response &resp) {
  if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
    ROS_WARN("[CEO]: cannot set default duty in auto mode");
    resp.success = (unsigned char) false;
    return true;
  } else {
    ROS_WARN("[CEO]: default duty was set!");
    default_duty_.duty = req.duty;
    default_duty_.current_task_idx = 0;
    default_duty_.complete = false;
    default_duty_.duty_set = true;
    for (int i = 0; i < default_duty_.duty.total_tasks; i++) {
      default_duty_.duty.tasks[i].status = mw_msgs::Task::NOT_STARTED;
      if (default_duty_.duty.tasks[i].type == mw_msgs::Task::ROUTE) {
        default_duty_.duty.tasks[i].route.header.stamp = ros::Time::now();
        default_duty_.duty.tasks[i].route.header.frame_id = "map";
        for (int j = 0 ; j < default_duty_.duty.tasks[i].route.waypoints.size() ; j++) {
          default_duty_.duty.tasks[i].route.waypoints[j].header = default_duty_.duty.tasks[i].route.header;
        }
      }
    }
    resp.success = (unsigned char) true;
    return true;
  }
}

bool ChiefExecutiveClass::setCurrentDuty(mw_msgs::SetDuty::Request &req, mw_msgs::SetDuty::Response &resp) {
  if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
    ROS_WARN("[CEO]: cannot set current duty in auto mode");
    resp.success = (unsigned char) false;
    return true;
  } else {
    ROS_WARN("[CEO]: current duty was set!");
    current_duty_.duty = req.duty;
    current_duty_.current_task_idx = 0;
    current_duty_.complete = false;
    current_duty_.duty_set = true;
    for (int i = 0; i < current_duty_.duty.total_tasks; i++) {
      current_duty_.duty.tasks[i].status = mw_msgs::Task::NOT_STARTED;
      if (current_duty_.duty.tasks[i].type == mw_msgs::Task::ROUTE) {
        current_duty_.duty.tasks[i].route.header.stamp = ros::Time::now();
        current_duty_.duty.tasks[i].route.header.frame_id = "map";
        for (int j = 0 ; j < current_duty_.duty.tasks[i].route.waypoints.size() ; j++) {
          current_duty_.duty.tasks[i].route.waypoints[j].header = current_duty_.duty.tasks[i].route.header;
        }
      }
    }
    resp.success = (unsigned char) true;
    return true;
  }
}

bool ChiefExecutiveClass::setCEOMode(mw_msgs::SetMode::Request &req, mw_msgs::SetMode::Response &resp) {
  if (robot_status_.chief_executive_mode == req.mode) {
    ROS_WARN("[CEO]: already in the required mode");
    resp.success = (unsigned char) true;
    return true;
  }
  switch (req.mode) {
    case mw_msgs::ChiefExecutiveMode::WAITING_FOR_AUTO_MODE:
      if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
        //save the current duty stuff for resuming again
      }
      robot_status_.chief_executive_mode = mw_msgs::ChiefExecutiveMode::WAITING_FOR_AUTO_MODE;
      resp.success = (unsigned char) true;
      break;
    case mw_msgs::ChiefExecutiveMode::AUTO_MODE:
      if (robot_status_.chief_executive_mode != mw_msgs::ChiefExecutiveMode::WAITING_FOR_AUTO_MODE) {
        ROS_WARN("[CEO]: cannot directly go to AUTO_MODE");
        resp.success = (unsigned char) false;
      } else {
        robot_status_.chief_executive_mode = mw_msgs::ChiefExecutiveMode::AUTO_MODE;
        resp.success = (unsigned char) true;
      }
      break;
    case mw_msgs::ChiefExecutiveMode::MAPPING_MODE:
      if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
        ROS_WARN("[CEO]: cannot directly go to MAPPING from AUTO_MODE");
        resp.success = (unsigned char) false;
      } else {
        //system calls needed to start the nodes needed for mapping
        robot_status_.chief_executive_mode = mw_msgs::ChiefExecutiveMode::MAPPING_MODE;
        resp.success = (unsigned char) true;
      }
      break;
    case mw_msgs::ChiefExecutiveMode::ROUTE_AND_DUTY_CREATION_MODE:
      if (robot_status_.chief_executive_mode == mw_msgs::ChiefExecutiveMode::AUTO_MODE) {
        ROS_WARN("[CEO]: cannot directly go to DUTY_CREATION from AUTO_MODE");
        resp.success = (unsigned char) false;
      } else {
        //system calls needed to start the nodes needed for DUTY_CREATION
        robot_status_.chief_executive_mode = mw_msgs::ChiefExecutiveMode::ROUTE_AND_DUTY_CREATION_MODE;
        resp.success = (unsigned char) true;
      }
      break;
    default: ROS_WARN("[CEO]: Invalid chief executive mode requested");
  }
  return true;
}

void ChiefExecutiveClass::executeCB(const ros::TimerEvent &event) {
  //Check if current duty exists
  if (current_duty_.duty_set) {
    robot_status_.current_duty_id = current_duty_.duty.id;
    robot_status_.current_task_id = current_duty_.current_task_idx;
    robot_status_.task_type = current_duty_.duty.tasks[current_duty_.current_task_idx].type;
  } else {
    robot_status_.current_task_id = 0;
    robot_status_.task_type = 0;
  }
  robot_status_.battery_status = battery_status_;
  robot_status_.error_code = robot_error_code_;
  status_pub_.publish(robot_status_);

  switch (robot_status_.chief_executive_mode) {
    case mw_msgs::ChiefExecutiveMode::WAITING_FOR_AUTO_MODE: ROS_INFO_THROTTLE(30, "[CEO]: in waiting for auto mode");
      //do all the sanity checks and ensure that system is good
      break;
    case mw_msgs::ChiefExecutiveMode::AUTO_MODE: ROS_INFO_THROTTLE(30, "[CEO]: in auto mode");
      if (!sanityForAutoMode()) {
        error_code_ = 1;
        break;
      }
      if (!processTask()) {
        error_code_ = 2;
      }
      break;

    case mw_msgs::ChiefExecutiveMode::MAPPING_MODE:
      //no idea as of now
      break;

    case mw_msgs::ChiefExecutiveMode::ROUTE_AND_DUTY_CREATION_MODE:
      //no idea as of now
      break;

    default:ROS_WARN("[CEO] : In the default mode: it should NOT be here");
      break;
  }
}

bool ChiefExecutiveClass::sanityForAutoMode() {
  if (!current_duty_.duty_set) {
    ROS_WARN_THROTTLE(5, "[CEO] : auto mode initiated without assigning a duty");
    return false;
  }
  if (current_duty_.duty.total_tasks < 2) {
    ROS_WARN_THROTTLE(5, "[CEO] : current duty has less than two tasks");
    return false;
  }
  if (current_duty_.current_task_idx > current_duty_.duty.total_tasks) {
    ROS_WARN_THROTTLE(5, "[CEO] : something is messed up with current duty");
    return false;
  }
  return true;
}

bool ChiefExecutiveClass::processTask() {
  ROS_INFO_THROTTLE(30,
                    "[CEO] : current_task: %d task status: %d",
                    current_duty_.current_task_idx,
                    current_duty_.duty.tasks[current_duty_.current_task_idx].status);
  switch (current_duty_.duty.tasks[current_duty_.current_task_idx].status) {
    case mw_msgs::Task::NOT_STARTED: ROS_INFO_THROTTLE(30, "[CEO]: in 'not yet started' mode");
      if (current_duty_.duty.tasks[current_duty_.current_task_idx].type == mw_msgs::Task::ROUTE) {
        //call the service of route executive to start the route
        mw_msgs::SetRoute srv;
        srv.request.route = current_duty_.duty.tasks[current_duty_.current_task_idx].route;
        ROS_INFO("[CEO]: called executive/set_route service");
        if (set_route_client_.call(srv)) {
          current_duty_.duty.tasks[current_duty_.current_task_idx].status = mw_msgs::Task::WIP;
          return true;
        }
        return false; //might change on the output of above line
      } else {
        //call the service of task executive to start the task
        mw_msgs::SetTask srv;
        srv.request.task = current_duty_.duty.tasks[current_duty_.current_task_idx];
        set_task_client_.call(srv);
        ROS_INFO("[CEO]: called task_executive/set_task service");
        if (srv.response.success) {
          current_duty_.duty.tasks[current_duty_.current_task_idx].status = mw_msgs::Task::WIP;
          return true;
        }
        return false;
      }

    case mw_msgs::Task::WIP: ROS_INFO_THROTTLE(30, "[CEO]: in WIP mode");
      if (current_duty_.duty.tasks[current_duty_.current_task_idx].type == mw_msgs::Task::ROUTE) {
        mw_msgs::GetRouteStatus srv;
        srv.request.id = current_duty_.duty.tasks[current_duty_.current_task_idx].route.id;
        srv.request.pause_state = (unsigned char) auto_pause_;
        route_status_client_.call(srv);
        if (srv.response.status == mw_msgs::Route::COMPLETED) {
          current_duty_.duty.tasks[current_duty_.current_task_idx].status = mw_msgs::Task::COMPLETED;
          current_duty_.complete = true;
          completeEndBehaviour(current_duty_.duty.tasks[current_duty_.current_task_idx].end_behaviour);
        }
        return true; //might change on the output of above line
      } else if (current_duty_.duty.tasks[current_duty_.current_task_idx].type != mw_msgs::Task::ROUTE) {
        mw_msgs::GetTaskStatus srv;
        srv.request.id = current_duty_.duty.tasks[current_duty_.current_task_idx].id;
        task_status_client_.call(srv);
        if (srv.response.status == mw_msgs::Task::COMPLETED) {
          current_duty_.duty.tasks[current_duty_.current_task_idx].status = mw_msgs::Task::COMPLETED;
          current_duty_.complete = true;
          completeEndBehaviour(current_duty_.duty.tasks[current_duty_.current_task_idx].end_behaviour);
        }
        // call the service of attachement executive to get the status of attachement
        // the status will change here when the task is complete
        // make sure to change it in vector and not in temp
        return true;
      } else {
        ROS_WARN("[CEO] : Invalid type of task");
        return false;
      }

    case mw_msgs::Task::COMPLETED: ROS_INFO_THROTTLE(5, "[CEO]: in 'complete' mode");
      if (current_duty_.current_task_idx == current_duty_.duty.total_tasks - 1) {
        if (current_duty_.duty.loop_enabled) {
          resetDutyMonitor();
        } else {
          ROS_INFO_THROTTLE(5, "[CEO]: task completed");
        }

      } else if (current_duty_.current_task_idx < current_duty_.duty.total_tasks) {
        current_duty_.current_task_idx++;
      }
      return true;

    default: ROS_WARN("[CEO]: In Default MODE: shouldn't be here");
      return false;
  }
}

void ChiefExecutiveClass::resetDutyMonitor() {
  current_duty_.complete = false;
  current_duty_.current_task_idx = 0;
  for (int i = 0; i < current_duty_.duty.total_tasks; i++) {
    current_duty_.duty.tasks[i].status = mw_msgs::Task::NOT_STARTED;
  }
}

void ChiefExecutiveClass::completeEndBehaviour(float duration) {
  if (duration < 0) {
    //put the code here to wait for manual acknowledgment
  } else {
    ros::Duration(duration).sleep();
  }
}

}