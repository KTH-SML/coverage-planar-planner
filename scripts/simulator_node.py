#! /usr/bin/env python


import rospy as rp
import coverage_planar_planner.msg as cms

import numpy as np


rp.init_node('simulator_node')
pose_pub = rp.Publisher('pose', cms.Pose, queue_size=10)

lin_vel = np.zeros(2)
ang_vel = 0.0
position = np.array(rp.get_param('initial_position', [0.0,0.0]))
yaw = rp.get_param('initial_yaw', 0.0)
time = rp.get_time()

def vel_cb(vel):
    global lin_vel, ang_vel
    lin_vel = np.array(vel.linear)
    ang_vel = vel.angular
    
vel_sub = rp.Subscriber('cmd_vel', cms.Velocity, vel_cb)

def work():
    global lin_vel, ang_vel
    global position, yaw
    global time
    new_time = rp.get_time()
    dt = new_time - time
    position += lin_vel*dt
    yaw += ang_vel*dt
    pose = cms.Pose(position=position, orientation=[np.cos(yaw), np.sin(yaw)])
    pose_pub.publish(pose)
    time = new_time
    
rate = rp.Rate(1e1)

while not rp.is_shutdown():
    work()
    rate.sleep()
