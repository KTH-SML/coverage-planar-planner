#! /usr/bin/env python


import rospy as rp

import coverage_planar_planner.msg as cms
import geometry_msgs.msg as gms

import utilities as uts
import tf.transformations as tft
import numpy as np


RADIUS = 1.0

rp.init_node('mobile_landmark_simulator_node')
pub = rp.Publisher('mobile_landmark', cms.Landmark, queue_size=10)
#gms_pose_pub = rp.Publisher('gms_pose', gms.Pose, queue_size=10)

def work():
    time = rp.get_time()
    angle = 2*np.pi*0.05*time
    x = np.cos(angle)
    y = np.sin(angle)
    position = RADIUS*np.array([x, y])
    orientation = np.array([x, y])
    lmk = cms.Landmark(position=position, orientation=orientation)
    pub.publish(lmk)

rate = rp.Rate(6e1)

while not rp.is_shutdown():
    work()
    rate.sleep()
