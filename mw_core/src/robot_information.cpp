#include <mw_core/robot_information.h>
#include <mw_core/utility_functions.h>

namespace mw_core
{

RobotInformation::RobotInformation(TF &tf_listener,
                                   const std::string &global_frame,
                                   const std::string &robot_frame,
                                   const ros::Duration &tf_timeout)
    : tf_listener_(tf_listener), global_frame_(global_frame), robot_frame_(robot_frame), tf_timeout_(tf_timeout)
{
}

bool RobotInformation::getRobotPose(geometry_msgs::PoseStamped &robot_pose) const
{
    bool tf_success = mw_core::getRobotPose(tf_listener_, robot_frame_, global_frame_,
                                                ros::Duration(tf_timeout_), robot_pose);
    robot_pose.header.stamp = ros::Time::now(); // would be 0 if not, as we ask tf listener for the last pose available
    if (!tf_success)
    {
        ROS_ERROR_STREAM("Can not get the robot pose in the global frame. - robot frame: \""
                         << robot_frame_ << "\"   global frame: \"" << global_frame_ << std::endl);
        return false;
    }
    return true;
}

bool RobotInformation::getRobotVelocity(geometry_msgs::TwistStamped &robot_velocity, ros::Duration look_back_duration) const
{
    // TODO implement and filter tf data to compute velocity.
    return false;
}

const std::string &RobotInformation::getGlobalFrame() const { return global_frame_; };

const std::string &RobotInformation::getRobotFrame() const { return robot_frame_; };

const TF &RobotInformation::getTransformListener() const { return tf_listener_; };

const ros::Duration &RobotInformation::getTfTimeout() const { return tf_timeout_; }

} // namespace mbf_abstract_nav