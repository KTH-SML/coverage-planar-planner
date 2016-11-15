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
#landmarks = set([lm.Landmark(ori=[-1,0.1])])
landmarks = set()
sensor = sn.Sensor(fp=fp.EggFootprint())



rp.init_node('planner_node')

KP = rp.get_param('position_gain', 1.0)
KN = rp.get_param('orientation_gain', 1.0)
SP = rp.get_param('velocity_saturation', 0.3)
SN = rp.get_param('angular_velocity_saturation', 0.3)

XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))

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












def add_landmark_arc_handler(req):
    global landmarks
    global lock
    num = req.number
    rd = req.radius
    th1 = req.thetamin
    th2 = req.thetamax
    lmks = set()
    for th in np.linspace(np.pi*th1,np.pi*th2,num):
        ori = np.array([np.cos(th), np.sin(th)])
        pos = rd*ori
        lmk = lm.Landmark(pos=pos, ori=ori)
        lmks.add(lmk)
    lock.acquire()
    landmarks |= lmks
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = None,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.AddLandmarkArcResponse()

add_lma_srv = rp.Service(
    'add_landmark_arc',
    csv.AddLandmarkArc,
    add_landmark_arc_handler)





def add_random_landmarks_handler(req):
    global landmarks
    global lock
    lmks = [lm.Landmark.random(
	    	xlim=0.3*np.array(XLIM),
	    	ylim=0.3*np.array(YLIM))
    	for index in range(req.num)]
    lock.acquire()
    landmarks |= set(lmks)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = None,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.AddRandomLandmarksResponse()

add_lmk_srv = rp.Service(
    'add_random_landmarks',
    csv.AddRandomLandmarks,
    add_random_landmarks_handler
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


def add_landmark_handler(req):
    global landmarks
    global lock
    lmk = lm.Landmark.from_msg(req.landmark)
    lock.acquire()
    landmarks.add(lmk)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = None,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.AddLandmarkResponse()
add_lmk_srv = rp.Service(
    'add_landmark',
    csv.AddLandmark,
    add_landmark_handler)


def change_landmarks_handler(req):
    global landmarks
    global lock
    lmks = [lm.Landmark.from_message(lmk_msg) for lmk_msg in req.landmarks]
    lock.acquire()
    landmarks = set(lmks)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = None,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.ChangeLandmarksResponse()
chg_lmk_srv = rp.Service(
    'change_landmarks',
    csv.ChangeLandmarks,
    change_landmarks_handler
)






#def smart_gain(coverage, gmin, gmax):
#    return gmin + 2*(gmax-gmin)/(1+np.exp(0.5*coverage))




def work():
    global sensor, landmarks
    global vel_pub, lmks_pub
    global KP, KN
    global lock
    lock.acquire()
    coverage = sensor.coverage(landmarks)
    p = sensor.pos
    n = sensor.ori
    if len(landmarks) == 0:
        v = np.zeros(2)
        w = 0.0
    else:
        #v = -smart_gain(coverage,KP/10,KP)*sensor.cov_pos_grad(landmarks)
        v = -KP/len(landmarks)*sensor.cov_pos_grad(landmarks)
        v = uts.saturate(v,SP)
        if p[0] <= XLIM[0] and v[0] <= 0.0 or p[0] >= XLIM[1] and v[0] >= 0.0:
            v[0] = 0.0
        if p[1] <= YLIM[0] and v[1] <= 0.0 or p[1] >= YLIM[1] and v[1] >= 0.0:
            v[1] = 0.0
        #w = -smart_gain(coverage,KN/10,KN)*np.cross(n, sensor.cov_ori_grad(landmarks))
        w = -KN/len(landmarks)*np.cross(n, sensor.cov_ori_grad(landmarks))
        w = uts.saturate(w, SN)
    coverage = sensor.coverage(landmarks)
    lmks_msg = [lmk.to_msg() for lmk in landmarks]
    lock.release()
    cov_vel = cms.Velocity(linear=v, angular=w)
    vel_pub.publish(cov_vel)
    #lmks_pub.publish(lmks_msg)
    cov_pub.publish(coverage)







while not rp.is_shutdown():
    work()
    RATE.sleep()
