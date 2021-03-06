/**
 * 
 * @file types.h
 * @brief Contains typedefs for tfs
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */
#pragma once

#if ROS_VERSION_MINIMUM(1, 12, 0) // if current ros version is >= 1.12.0
// Melodic uses TF2
#include <tf2_ros/buffer.h>
typedef boost::shared_ptr<tf2_ros::Buffer> TFPtr;
typedef tf2_ros::Buffer TF;
typedef tf2::TransformException TFException;
#else
// Previous versions still using TF
#define USE_OLD_TF
#include <tf/transform_listener.h>
typedef boost::shared_ptr<tf::TransformListener> TFPtr;
typedef tf::TransformListener TF;
typedef tf::TransformException TFException;

#endif
