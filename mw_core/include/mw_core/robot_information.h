#pragma once

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/duration.h>
#include <string>
#include <tf/transform_listener.h>

#include <mw_core/types.h>

namespace mw_core
{

class RobotInformation
{

public:
    typedef boost::shared_ptr<RobotInformation> Ptr;

    RobotInformation(
        TF &tf_listener,
        const std::string &global_frame,
        const std::string &robot_frame,
        const ros::Duration &tf_timeout);

    bool getRobotPose(geometry_msgs::PoseStamped &robot_pose) const;

    bool getRobotVelocity(geometry_msgs::TwistStamped &robot_velocity, ros::Duration look_back_duration) const;

    const std::string &getGlobalFrame() const;

    const std::string &getRobotFrame() const;

    const TF &getTransformListener() const;

    const ros::Duration &getTfTimeout() const;

private:
    const TF &tf_listener_;

    const std::string &global_frame_;

    const std::string &robot_frame_;

    const ros::Duration &tf_timeout_;
};

} 