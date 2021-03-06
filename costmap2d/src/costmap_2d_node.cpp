/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>


void global_costmap_thread(tf2_ros::Buffer *buffer){
  costmap_2d::Costmap2DROS global_costmap("global_costmap", *buffer);
  global_costmap.start();
  while(ros::ok()){
    global_costmap.spinClass();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
  ROS_INFO("Finishing thread for global costmap");
}

void local_costmap_thread(tf2_ros::Buffer *buffer){
  costmap_2d::Costmap2DROS local_costmap("local_costmap", *buffer);
  local_costmap.start();
  while (ros::ok()){
    local_costmap.spinClass();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
  ROS_INFO("Finishing thread for local costmap");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  boost::thread t1(global_costmap_thread,&buffer);
  boost::thread t2(local_costmap_thread, &buffer);
  ROS_INFO("Costmap node has started");
  while(ros::ok()){
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  t1.join();
  t2.join();
  return (0);}
