#!/usr/bin/env python

import sys
import copy
import rospy

import numpy as np

import datetime
from std_msgs.msg import String, Header, Int64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Wrench, WrenchStamped, Quaternion, Transform
from sensor_msgs.msg import JointState, PointCloud2

import tf
from tf import TransformListener

import csv

from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from moveit_commander.conversions import pose_to_list
import moveit_commander


class tfToPose():
    def __init__(self):

        rospy.Subscriber("/joint_states",                           JointState,             self.jointStateCallback, queue_size=1)

        self._pose_publisher = rospy.Publisher("/link_6_pose", PoseStamped, queue_size=1)

        self._tf = TransformListener()

        self._tempValue_jointState              = JointState()        
        self._robotJointState = JointState()
        self._poseArray = PoseArray()


    def jointStateCallback(self, msg):
        self._tempValue_jointState = copy.deepcopy(msg)

        (trans,rot) = self._tf.lookupTransform("base_link", "link_6", rospy.Duration(0))

        msg2pub = PoseStamped()
        msg2pub.header.stamp = rospy.Time.now()
        msg2pub.header.frame_id = 'base_link'
        msg2pub.pose.position.x = trans[0]
        msg2pub.pose.position.y = trans[1]
        msg2pub.pose.position.z = trans[2]
        msg2pub.pose.orientation.x = rot[0]
        msg2pub.pose.orientation.y = rot[1]
        msg2pub.pose.orientation.z = rot[2]
        msg2pub.pose.orientation.w = rot[3]

        self._pose_publisher.publish(msg2pub)

    def run(self):
        while not rospy.is_shutdown():

            pass

if __name__ == '__main__':

    rospy.init_node('tf_to_pose')
    node = tfToPose()

    node.run()

    rospy.spin()


