#! /usr/bin/env python


import rospy as rp
import coverage_planar_planner.msg as cms

import numpy as np
import matplotlib.pyplot as plt
import threading as thd

import landmark as lm
import sensor as sn

rp.init_node('plotter_node')
XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))
NAMES = rp.get_param('names', 'Axel Bo').split()
COLORS = rp.get_param('colors', 'blue red').split()
COLDIC = dict()
for index, name in enumerate(NAMES):
    COLDIC[name] = COLORS[index]


plt.ion()
fig = plt.figure(figsize=(10,10))
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.axis('equal')
plt.axis(XLIM+YLIM)
plt.grid()

lock = thd.Lock()

sensors = dict()
draw_sensor_flags = dict()
for name in NAMES:
    sensors[name] = sn.Sensor(color=COLDIC[name])
    draw_sensor_flags[name] = True

def pose_cb(pose, name):
    global sensors
    global draw_sensor_flags
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    lock.acquire()
    sensors[name].pos = p
    sensors[name].ori = n
    draw_sensor_flags[name] = True
    lock.release()

pose_subs = [
    rp.Subscriber(
        name+'/pose',
        cms.Pose,
        pose_cb,
        callback_args=name)
    for name in NAMES]


landmarks = dict()
draw_landmarks_flags = dict()
for name in NAMES:
    landmarks[name] = set()
    draw_landmarks_flags[name] = True
    
def landmarks_cb(msg, name):
    global landmarks
    global draw_landmarks_flags
    lock.acquire()
    landmarks[name] = [lm.Landmark.from_msg(datum) for datum in msg.data]
    draw_landmarks_flags[name] = True
    lock.release()

landmarks_subs = [
    rp.Subscriber(
        name+'/landmarks',
        cms.LandmarkArray,
        landmarks_cb,
        callback_args=name)
    for name in NAMES
]

points = dict()
arrows = dict()
lmks_artists = dict()
for name in NAMES:
    points[name], arrows[name] = sensors[name].draw()
    lmks_artists[name] = [
        lmk.draw(
            color=COLDIC[name],
            draw_orientation=False,
            scale=1.0)
        for lmk in landmarks[name]]
def work():
    global points, arrows, lmks_artists
    global draw_sensor_flags, draw_landmarks_flags
    global sensors, landmarks
    lock.acquire()
    for name in NAMES:
        if draw_sensor_flags[name]:
            points[name].remove()
            if not arrows[name] == None:
                arrows[name].remove()
            points[name], arrows[name] = sensors[name].draw()
            draw_sensor_flags[name] = False
        if draw_landmarks_flags[name]:
            for lp, la in lmks_artists[name]:
                lp.remove()
                if not la == None:
                    la.remove()
            lmks_artists[name] = [
                lmk.draw(
                    color=COLDIC[name],
                    draw_orientation=False,
                    scale=5.0)
                for lmk in landmarks[name]]
            draw_landmarks_flags[name] = False
    lock.release()
    plt.draw()




rate = rp.Rate(6e1)
if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
        rate.sleep()
