#!/usr/bin/env python
'''
@brief for publishing odom msg based on pose
'''

from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib
import rospy
from copy import deepcopy

# specific to the controller under testing
import mw_msgs.msg
from mw_msgs.msg import MotionControllerGoal
# import MotionController.msg

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import degrees, radians

# for debuging and user interface
import pdb
import argparse



class odomGen:
    def __init__(self, posetopic = '/pose', child_frame_id = 'base_link'):

        self.pose_initialized_ = False
        self.pose_topic_ = posetopic
        self.odom_topic_ = "/odom"
        self.child_frame_id_ = child_frame_id

        rospy.init_node('pose_2_odom', anonymous=True)

        self.poseSub_ = rospy.Subscriber(self.pose_topic_, PoseStamped, self.poseCB)
        self.odomPub_ = rospy.Publisher(self.odom_topic_, Odometry, queue_size=10)

        self.prev_pose_ = PoseStamped()
        self.curr_pose_ = PoseStamped()
        self.prev_rpy_ = []
        self.curr_rpy_ = []


    def poseCB(self, pose_in):
        self.prev_pose_ = deepcopy(self.curr_pose_)
        self.curr_pose_ = deepcopy(pose_in)
        if self.curr_pose_.header.stamp.is_zero():
            # rospy.logdebug("POSE_2_ODOM: no time stamps in pose msg,  using current time as time stamp")
            self.curr_pose_.header.stamp = rospy.Time.now()
        self.prev_rpy_ = deepcopy(self.curr_rpy_)
        

        q_curr = (self.curr_pose_.pose.orientation.x,
             self.curr_pose_.pose.orientation.y,
             self.curr_pose_.pose.orientation.z,
             self.curr_pose_.pose.orientation.w)

        self.curr_rpy_ = euler_from_quaternion(q_curr)

        if not self.pose_initialized_:
            self.pose_initialized_ = True
            return

        delta_t = self.curr_pose_.header.stamp.to_sec() - self.prev_pose_.header.stamp.to_sec()

        vx = (self.curr_pose_.pose.position.x - self.prev_pose_.pose.position.x)/delta_t
        vy = (self.curr_pose_.pose.position.y - self.prev_pose_.pose.position.y)/delta_t
        vz = (self.curr_pose_.pose.position.z - self.prev_pose_.pose.position.z)/delta_t

        vrol = (self.curr_rpy_[0] - self.prev_rpy_[0])/delta_t
        vpit = (self.curr_rpy_[1] - self.prev_rpy_[1])/delta_t
        vyaw = (self.curr_rpy_[2] - self.prev_rpy_[2])/delta_t

        odom_msg = Odometry()
        odom_msg.header = self.curr_pose_.header
        odom_msg.child_frame_id = self.child_frame_id_

        odom_msg.pose.pose = deepcopy(self.curr_pose_.pose)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz

        odom_msg.twist.twist.angular.x = vrol
        odom_msg.twist.twist.angular.y = vpit
        odom_msg.twist.twist.angular.z = vyaw

        self.odomPub_.publish(odom_msg)


if __name__ =='__main__':

    odomGen()
    rospy.spin()
