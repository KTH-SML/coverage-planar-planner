#! /usr/bin/env python


import rospy as rp
import coverage_planar_planner.msg as cms
import coverage_planar_planner.srv as csv

import numpy as np
import matplotlib.pyplot as plt
import threading as thd

import landmark as lm
import sensor as sn

rp.init_node('plotter_node')
XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))

plt.ion()
fig = plt.figure()
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.axis('equal')
plt.axis(XLIM+YLIM)
plt.grid()

sensor_lock = thd.Lock()
landmarks_lock = thd.Lock()

sensor = sn.Sensor()
draw_sensor_flag = True
def pose_cb(pose):
    global sensor, sensor_lock
    global draw_sensor_flag
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    sensor_lock.acquire()
    sensor.pos = p
    sensor.ori = n
    draw_sensor_flag = True
    sensor_lock.release()
pose_sub = rp.Subscriber('pose', cms.Pose, pose_cb)


landmarks = set()
draw_landmarks_flag = True
# def landmarks_cb(msg):
#     global landmarks
#     global draw_landmarks_flag
#     lock.acquire()
#     landmarks = [lm.Landmark.from_msg(datum) for datum in msg.data]
#     draw_landmarks_flag = True
#     lock.release()
# landmarks_sub = rp.Subscriber('landmarks', cms.LandmarkArray, landmarks_cb)


def draw_landmarks_handler(msg):
    global landmarks
    global draw_landmarks_flag
    global landmarks_lock
    name = msg.name
    landmarks_lock.acquire()
    landmarks = [lm.Landmark.from_msg(datum) for datum in msg.landmarks]
    draw_landmarks_flag = True
    landmarks_lock.release()
    return csv.DrawLandmarksResponse()

draw_landmarks_service = rp.Service(
    'draw_landmarks',
    csv.DrawLandmarks,
    draw_landmarks_handler)





point, arrow = sensor.draw()
lmks_artists = [lmk.draw() for lmk in landmarks]
def work():
    global point, arrow, lmks_artists
    global draw_sensor_flag, draw_landmarks_flag
    global sensor, sensor_lock
    global landmarks, landmarks_lock
    sensor_lock.acquire()
    if draw_sensor_flag:
        point.remove()
        if not arrow == None:
            arrow.remove()
        point, arrow = sensor.draw()
        draw_sensor_flag = False
    sensor_lock.release()
    landmarks_lock.acquire()
    if draw_landmarks_flag:
        for lp, la in lmks_artists:
            lp.remove()
            if not la == None:
                la.remove()
        lmks_artists = [lmk.draw(draw_orientation=False) for lmk in landmarks]
        draw_landmarks_flag = False
    landmarks_lock.release()
    plt.draw()




rate = rp.Rate(6e1)
if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
        rate.sleep()
