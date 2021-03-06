/**
 * 
 * @file utility_functions.cpp
 * @brief This file contains implementations of helper functions for the navigation stack defined in utility_functions.h.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#include <mw_core/utility_functions.h>

namespace mw_core
{
bool getRobotPose(const TF &tf,
                  const std::string &robot_frame,
                  const std::string &global_frame,
                  const ros::Duration &timeout,
                  geometry_msgs::PoseStamped &robot_pose)
{
    tf::Stamped<tf::Pose> local_pose;
    local_pose.setIdentity();
    local_pose.frame_id_ = robot_frame;
    geometry_msgs::PoseStamped local_pose_msg;
    tf::poseStampedTFToMsg(local_pose, local_pose_msg);
    return transformPose(tf,
                         global_frame,
                         ros::Time(0), // most recent available
                         timeout,
                         local_pose_msg,
                         global_frame,
                         robot_pose);
}
bool transformPose(const TF &tf,
                   const std::string &target_frame,
                   const ros::Time &target_time,
                   const ros::Duration &timeout,
                   const geometry_msgs::PoseStamped &in,
                   const std::string &fixed_frame,
                   geometry_msgs::PoseStamped &out)
{
    std::string error_msg;

#ifdef USE_OLD_TF
    bool success = tf.waitForTransform(target_frame,
                                       target_time,
                                       in.header.frame_id,
                                       in.header.stamp,
                                       fixed_frame,
                                       timeout,
                                       ros::Duration(0.01),
                                       &error_msg);
#else
    bool success = tf.canTransform(target_frame,
                                   target_time,
                                   in.header.frame_id,
                                   in.header.stamp,
                                   fixed_frame,
                                   timeout,
                                   &error_msg);
#endif

    if (!success)
    {
        ROS_WARN_STREAM("Failed to look up transform from frame '" << in.header.frame_id << "' into frame '" << target_frame
                                                                   << "': " << error_msg);
        return false;
    }

    try
    {
#ifdef USE_OLD_TF
        tf.transformPose(target_frame, target_time, in, fixed_frame, out);
#else
        tf.transform(in, out, target_frame, target_time, fixed_frame);
#endif
    }
    catch (const TFException &ex)
    {
        ROS_WARN_STREAM("Failed to transform pose from frame '" << in.header.frame_id << " ' into frame '"
                                                                << target_frame << "' with exception: " << ex.what());
        return false;
    }
    return true;
}

bool transformPoint(const TF &tf,
                    const std::string &target_frame,
                    const ros::Time &target_time,
                    const ros::Duration &timeout,
                    const geometry_msgs::PointStamped &in,
                    const std::string &fixed_frame,
                    geometry_msgs::PointStamped &out)
{
    std::string error_msg;

#ifdef USE_OLD_TF
    bool success = tf.waitForTransform(target_frame,
                                       target_time,
                                       in.header.frame_id,
                                       in.header.stamp,
                                       fixed_frame,
                                       timeout,
                                       ros::Duration(0.01),
                                       &error_msg);
#else
    bool success = tf.canTransform(target_frame,
                                   target_time,
                                   in.header.frame_id,
                                   in.header.stamp,
                                   fixed_frame,
                                   timeout,
                                   &error_msg);
#endif

    if (!success)
    {
        ROS_WARN_STREAM("Failed to look up transform from frame '" << in.header.frame_id << "' into frame '" << target_frame
                                                                   << "': " << error_msg);
        return false;
    }

    try
    {
#ifdef USE_OLD_TF
        tf.transformPoint(target_frame, target_time, in, fixed_frame, out);
#else
        tf.transform(in, out, target_frame, target_time, fixed_frame);
#endif
    }
    catch (const TFException &ex)
    {
        ROS_WARN_STREAM("Failed to transform point from frame '" << in.header.frame_id << " ' into frame '"
                                                                 << target_frame << "' with exception: " << ex.what());
        return false;
    }
    return true;
}

double distance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
    const geometry_msgs::Point &point1 = pose1.pose.position;
    const geometry_msgs::Point &point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    const double dz = point1.z - point2.z;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    //ROS_INFO("[Utility Functions]: dist is %lf",dist);
    return dist;
}

double distance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
  const geometry_msgs::Point &point1 = pose1.position;
  const geometry_msgs::Point &point2 = pose2.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  const double dz = point1.z - point2.z;
  double dist = sqrt(dx * dx + dy * dy + dz * dz);
  //ROS_INFO("[Utility Functions]: dist is %lf",dist);
  return dist;
}

double distance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  const double dz = point1.z - point2.z;
  double dist = sqrt(dx * dx + dy * dy + dz * dz);
  //ROS_INFO("[Utility Functions]: dist is %lf",dist);
  return dist;
}

double angle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
    const geometry_msgs::Quaternion &quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion &quaternion2 = pose2.pose.orientation;
    tf::Quaternion rotation1, rotation2;
    tf::quaternionMsgToTF(quaternion1, rotation1);
    tf::quaternionMsgToTF(quaternion2, rotation2);
    return rotation1.angleShortestPath(rotation2);
}

double angle(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
    const geometry_msgs::Quaternion &quaternion1 = pose1.orientation;
    const geometry_msgs::Quaternion &quaternion2 = pose2.orientation;
    tf::Quaternion rotation1, rotation2;
    tf::quaternionMsgToTF(quaternion1, rotation1);
    tf::quaternionMsgToTF(quaternion2, rotation2);
    return rotation1.angleShortestPath(rotation2);
}

void convertPointToPose(std::vector<Point> &points, std::vector<geometry_msgs::PoseStamped> &plan){
    geometry_msgs::Pose tempPose;
    geometry_msgs::PoseStamped pose;
    tf2::Quaternion tempQuat;
    for(int i=0;i<points.size();i++){
        //ROS_INFO("[Utility functions] i:%d,head:%lf",i,points[i].heading);
        tempQuat.setRPY(0,0,points[i].heading);
        tempQuat.normalize();
        geometry_msgs::Quaternion msgQuat;
        msgQuat = tf2::toMsg(tempQuat);
        tempPose.orientation = msgQuat;
        tempPose.position.x = points[i].x;
        tempPose.position.y = points[i].y;
        tempPose.position.z = 0;
        pose.pose = tempPose;
        ros::Time::init();
        pose.header.stamp = ros::Time::now();
        plan.push_back(pose);
    }
}

double yawFromQuaternion(const geometry_msgs::Quaternion & quaternion) {
  tf::Quaternion q(
    quaternion.x,
    quaternion.y,
    quaternion.z,
    quaternion.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double yawFromQuaternion(const geometry_msgs::Pose & pose) {
  tf::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion quaternionFromYaw(const double & yaw) {
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  tf::Matrix3x3 m(q);
  m.getRotation(q);
  geometry_msgs::Quaternion rot;
  rot.x = q.x();
  rot.y = q.y();
  rot.z = q.z();
  rot.w = q.w();
  return rot;
}

} // namespace mw_core