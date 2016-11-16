#! /usr/bin/env python
"""This node plots the sensors and the landmarks with matplotlib.

Subscriptions:
    - <name>/pose for <name> in rp.get_param('names').split()

Services offered:
    - draw_landmarks
"""

import rospy as rp
import coverage_planar_planner.msg as cms
import coverage_planar_planner.srv as csv

import numpy as np
import matplotlib.pyplot as plt
import threading as thd

import landmark as lm
import sensor as sn



rp.init_node('plotter_node')
XLIM = [float(elem) for elem in rp.get_param('xlim', "-0.4 2.4").split()]
YLIM = [float(elem) for elem in rp.get_param('ylim', "-2.0 1.4").split()]
NAMES = rp.get_param('names').split()
COLORS = rp.get_param('colors', '#1f618d #cb4335 #b7950b #659D32').split()
COLDIC = dict()
for index, name in enumerate(NAMES):
    COLDIC[name] = COLORS[index]



plt.ion()
fig = plt.figure(figsize=(10,10))
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.axis('equal')
plt.axis([XLIM[0]-2.0, XLIM[1]+2.0, YLIM[0]-2.0, YLIM[1]+2.0])
plt.grid()


sensor_locks = dict()
landmarks_locks = dict()
for name in NAMES:
    sensor_locks[name] = thd.Lock()
    landmarks_locks[name] = thd.Lock()


sensors = dict()
draw_sensor_flags = dict()
for name in NAMES:
    sensors[name] = sn.Sensor(color=COLDIC[name])
    draw_sensor_flags[name] = True




def pose_cb(pose, name):
    global sensors
    global draw_sensor_flags
    global sensor_locks
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    sensor_locks[name].acquire()
    sensors[name].pos = p
    sensors[name].ori = n
    draw_sensor_flags[name] = True
    sensor_locks[name].release()

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

#def landmarks_cb(msg, name):
#    global landmarks
#    global draw_landmarks_flags
#    global landmarks_locks
#    landmarks_locks[name].acquire()
#    landmarks[name] = [lm.Landmark.from_msg(datum) for datum in msg.data]
#    draw_landmarks_flags[name] = True
#    landmarks_locks[name].release()

#landmarks_subs = [
#    rp.Subscriber(
#        name+'/landmarks',
#        cms.LandmarkArray,
#        landmarks_cb,
#        callback_args=name)
#    for name in NAMES
#]



def draw_landmarks_handler(msg):
    global landmarks
    global draw_landmarks_flags
    global landmarks_locks
    name = msg.name
    landmarks_locks[name].acquire()
    landmarks[name] = [lm.Landmark.from_msg(datum) for datum in msg.landmarks]
    draw_landmarks_flags[name] = True
    landmarks_locks[name].release()
    return csv.DrawLandmarksResponse()

draw_landmarks_service = rp.Service(
    '/draw_landmarks',
    csv.DrawLandmarks,
    draw_landmarks_handler)






points = dict()
arrows = dict()
lmks_artists = dict()
for name in NAMES:
    sensor_locks[name].acquire()
    points[name], arrows[name] = sensors[name].draw()
    sensor_locks[name].release()
    landmarks_locks[name].acquire()
    lmks_artists[name] = [
        lmk.draw(
            color=COLDIC[name],
            draw_orientation=False,
            scale=1.0)
        for lmk in landmarks[name]]
    landmarks_locks[name].release()



save_times = [0.0, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0] + [
    10.0+5.0*i for i in range(20)]
save_counter = 0
def work():
    global points, arrows, lmks_artists
    global draw_sensor_flags, draw_landmarks_flags
    global sensors, landmarks
    global sensor_locks, landmarks_locks
    global save_interval, last_save_time, save_counter, initial_time
    for name in NAMES:
        sensor_locks[name].acquire()
        if draw_sensor_flags[name]:
            points[name].remove()
            if not arrows[name] == None:
                arrows[name].remove()
            points[name], arrows[name] = sensors[name].draw()
            draw_sensor_flags[name] = False
        sensor_locks[name].release()
        landmarks_locks[name].acquire()
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
        landmarks_locks[name].release()
    plt.draw()
    if save_counter < len(save_times):
        time = rp.get_time()-initial_time
        ths = save_times[save_counter]
        if time > save_times[save_counter]:
            save_counter += 1
            plt.savefig("coverageplot" + str(int(10*ths)) + ".pdf")




rate = rp.Rate(1e1)
if __name__ == '__main__':
    initial_time = rp.get_time()
    while not rp.is_shutdown():
        work()
        rate.sleep()
