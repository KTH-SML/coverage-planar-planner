#! /usr/bin/env python
import rospy as rp
import qualisys.msg as qms
import coverage_planar_planner.msg as cms

import numpy as np
import tf.transformations as tft



rp.init_node('qualisys_bridge')
rp.logwarn('Starting the qualysis bridge')

MAV_NAME = rp.get_param('mav_name', 'morph')
ALTITUDE = rp.get_param('altitude', 1.0)


pose_pub = rp.Publisher(
    'pose',
    cms.Pose,
    queue_size=10)


def subject_callback(msg):
    global pose_lock, saved_pose
    pose = cms.Pose()
    pose.position[0] = msg.position.x
    pose.position[1] = msg.position.y
    quaternion = np.array([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ])
    matrix = tft.quaternion_matrix(quaternion)
    pose.orientation = np.array(matrix[0:2,0])
    pose_pub.publish(pose)



pose_sub = rp.Subscriber(
    '/qualisys/'+MAV_NAME,
    qms.Subject,
    subject_callback)


rp.spin()
