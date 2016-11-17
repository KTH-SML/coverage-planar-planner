#! /usr/bin/env python


import rospy as rp
import coverage_planar_planner.msg as cms
import coverage_planar_planner.srv as csv
import std_msgs.msg as sms

import numpy as np
import threading as thd
import json
import os

import landmark as lm
import sensor as sn
import footprints as fp
import utilities as uts



lock = thd.Lock()
landmark = lm.Landmark()
sensor = sn.Sensor(fp=fp.EggFootprint())



rp.init_node('mobile_landmark_follower_node')



XLIM = [float(elem) for elem in rp.get_param('xlim', "-0.4 2.4").split()]
YLIM = [float(elem) for elem in rp.get_param('ylim', "-2.0 1.4").split()]
SCALE = (XLIM[1]-XLIM[0]+YLIM[1]-YLIM[0])*0.5


KP = rp.get_param('position_gain', 0.3*SCALE)
KN = rp.get_param('orientation_gain', 1.0)
SP = rp.get_param('velocity_saturation', 0.05*SCALE)
SN = rp.get_param('angular_velocity_saturation', 0.3)

vel_pub = rp.Publisher('cmd_vel', cms.Velocity, queue_size=10)
#lmks_pub = rp.Publisher('landmarks', cms.LandmarkArray, queue_size=10)
cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)

RATE = rp.Rate(6e1)

start_flag = False

rp.wait_for_service('/draw_landmarks')
draw_landmarks_proxy = rp.ServiceProxy(
    '/draw_landmarks',
    csv.DrawLandmarks)








def start_planner_handler(msg):
    global start_flag
    lock.acquire()
    start_flag = True
    lock.release()
    return csv.StartPlannerResponse()

rp.Service(
    'start_planner',
    csv.StartPlanner,
    start_planner_handler
)


lock.acquire()
while not rp.is_shutdown() and not start_flag:
    lock.release()
    RATE.sleep()
    lock.acquire()
lock.release()




def mobile_landmark_callback(msg):
    global landmark
    lock.acquire()
    landmark = lm.Landmark.from_msg(msg)
    lock.release()

rp.Subscriber(
    'mobile_landmark',
    cms.Landmark,
    mobile_landmark_callback
)







def change_gains_handler(req):
    global KP, KN
    global lock
    lock.acquire()
    KP = req.position_gain
    KN = req.orientation_gain
    lock.release()
    return csv.ChangeGainsResponse()
chg_gns_srv = rp.Service(
    'change_gains',
    csv.ChangeGains,
    change_gains_handler)


def pose_cb(pose):
    global sensor
    global lock
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    lock.acquire()
    sensor.pos = p
    sensor.ori = n
    lock.release()
pose_sub = rp.Subscriber(
    'pose',
    cms.Pose,
    pose_cb)







#def smart_gain(coverage, gmin, gmax):
#    return gmin + 2*(gmax-gmin)/(1+np.exp(0.5*coverage))




def work():
    global sensor, landmark
    global vel_pub, lmks_pub
    global KP, KN
    global lock
    lock.acquire()
    coverage = sensor.perception(landmark)
    p = sensor.pos
    n = sensor.ori
    #v = -smart_gain(coverage,KP/10,KP)*sensor.cov_pos_grad(landmarks)
    v = -KP*sensor.per_pos_grad(landmark)
    v = uts.saturate(v,SP)
    if p[0] <= XLIM[0] and v[0] <= 0.0 or p[0] >= XLIM[1] and v[0] >= 0.0:
        v[0] = 0.0
    if p[1] <= YLIM[0] and v[1] <= 0.0 or p[1] >= YLIM[1] and v[1] >= 0.0:
        v[1] = 0.0
    #w = -smart_gain(coverage,KN/10,KN)*np.cross(n, sensor.cov_ori_grad(landmarks))
    w = -KN*np.cross(n, sensor.per_ori_grad(landmark))
    w = uts.saturate(w, SN)
    coverage = sensor.perception(landmark)
    lmks_msg = [landmark.to_msg()]
    req = csv.DrawLandmarksRequest(
        name=None,
        landmarks=lmks_msg
    )
    draw_landmarks_proxy.call(req)
    lock.release()
    cov_vel = cms.Velocity(linear=v, angular=w)
    vel_pub.publish(cov_vel)
    #lmks_pub.publish(lmks_msg)
    cov_pub.publish(coverage)







while not rp.is_shutdown():
    work()
    RATE.sleep()
