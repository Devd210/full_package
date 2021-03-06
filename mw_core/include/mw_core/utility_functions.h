/**
 * 
 * @file utility_functions.h
 * @brief This file contains helper functions for the navigation stack.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 * 
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mw_core/types.h>
#include <mw_core/data_types/geometry_types.h>

namespace mw_core
{

/**
 * @brief Transforms a point from one frame into another.
 * @param tf_listener TransformListener.
 * @param target_frame Target frame for the point.
 * @param target_time Time, in that the frames should be used.
 * @param timeout Timeout for looking up the transformation.
 * @param in Point to transform.
 * @param fixed_frame Fixed frame of the source and target frame.
 * @param out Transformed point.
 * @return true, if the transformation succeeded.
 */
bool transformPoint(const TF &tf,
                    const std::string &target_frame,
                    const ros::Time &target_time,
                    const ros::Duration &timeout,
                    const geometry_msgs::PointStamped &in,
                    const std::string &fixed_frame,
                    geometry_msgs::PointStamped &out);

/**
 * @brief Transforms a pose from one frame into another.
 * @param tf_listener TransformListener.
 * @param target_frame Target frame for the pose.
 * @param target_time Time, in that the frames should be used.
 * @param timeout Timeout for looking up the transformation.
 * @param in Pose to transform.
 * @param fixed_frame Fixed frame of the source and target frame.
 * @param out Transformed pose.
 * @return true, if the transformation succeeded.
 */
bool transformPose(const TF &tf,
                   const std::string &target_frame,
                   const ros::Time &target_time,
                   const ros::Duration &timeout,
                   const geometry_msgs::PoseStamped &in,
                   const std::string &fixed_frame,
                   geometry_msgs::PoseStamped &out);

/**
 * @brief Computes the robot pose.
 * @param tf_listener TransformListener.
 * @param robot_frame frame of the robot.
 * @param global_frame global frame in which the robot is located.
 * @param timeout Timeout for looking up the transformation.
 * @param robot_pose the computed rebot pose in the global frame.
 * @return true, if succeeded, false otherwise.
 */
bool getRobotPose(const TF &tf,
                  const std::string &robot_frame,
                  const std::string &global_frame,
                  const ros::Duration &timeout,
                  geometry_msgs::PoseStamped &robot_pose);
/**
 * @brief Computes the Euclidean-distance between two poses.
 * @param pose1 pose 1
 * @param pose2 pose 2
 * @return Euclidean distance between pose 1 and pose 2.
 */
double distance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);
double distance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
double distance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);

/**
 * @brief computes the smallest angle between two poses.
 * @param pose1 pose 1
 * @param pose2 pose 2
 * @return smallest angle between pose 1 and pose 2.
 */
double angle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

double angle(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

/**
 * @brief Converts a set of Points into a set of poses
 * @param points Set of Points to convert
 * @param plan Set of poses to be returned
 */
void convertPointToPose(std::vector<Point> &points,std::vector<geometry_msgs::PoseStamped> &plan);

double yawFromQuaternion(const geometry_msgs::Quaternion & quaternion);
double yawFromQuaternion(const geometry_msgs::Pose & pose);

geometry_msgs::Quaternion quaternionFromYaw(const double & yaw);
} // namespace mw_core
