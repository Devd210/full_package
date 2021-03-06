//
// Created by naivehobo on 3/4/20.
//

#ifndef CONTROLLER_EXECUTIVE_EXCEPTION_H
#define CONTROLLER_EXECUTIVE_EXCEPTION_H

#include <exception>

namespace trajectory_planner {

class TrajectoryPlannerException : public std::exception {

 private:
  int error_;

 public:

  explicit TrajectoryPlannerException(int error) : error_(error) {}

  int getErrorCode() {
    return error_;
  }

  const char *what() const noexcept override {
    switch (error_) {
      case 0: return "All is well.";
      case 1: return "Could not call costmap service.";
      default: return "Unknown ERROR CODE";
    }
  }
};



}

#endif //CONTROLLER_EXECUTIVE_EXCEPTION_H
