#! /usr/bin/env python

import rospy as rp
import nav_msgs.msg as nms
import coverage_planar_planner.msg as cms

import numpy as np
import tf.transformations as tft


rp.init_node('odometry_to_landmark_bridge')

pub = rp.Publisher('mobile_landmark', cms.Landmark, queue_size=10)

def odometry_callback(msg):
    pose = cms.Landmark()
    pose.position[0] = msg.pose.pose.position.x
    pose.position[1] = msg.pose.pose.position.y
    quaternion = np.array([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])
    matrix = tft.quaternion_matrix(quaternion)
    pose.orientation = np.array(matrix[0:2,0])
    pub.publish(pose)

rp.Subscriber('mobile_landmark_odometry', nms.Odometry, odometry_callback)


rp.spin()
