#! /usr/bin/env python


import rospy as rp
import coverage_planar_planner.msg as cms
import coverage_planar_planner.srv as csv
import std_msgs.msg as sms

import numpy as np
import threading as thd
import random as rdm

import landmark as lm
import sensor as sns
import footprints as fp
import utilities as uts
import initial_landmarks as init_lmks
import initial_sensors as init_sns


landmarks = set()
landmarks_lock = thd.Lock()

incoming_landmarks = set()
incoming_landmarks_lock = thd.Lock()

#sensor = sns.Sensor(fp=fp.EggFootprint())
sensor_lock = thd.Lock()


rp.init_node('planner_node')

kp = rp.get_param('position_gain', 3.0)
kn = rp.get_param('orientation_gain', 1.0)
sp = rp.get_param('velocity_saturation', 1.0)
sn = rp.get_param('angular_velocity_saturation', 0.3)

NAMES = rp.get_param('/names').split()
MY_NAME = rp.get_param('name')
PARTNERS = filter(lambda x: not x == MY_NAME, NAMES)
possible_partners = list(PARTNERS)
landmarks = init_lmks.LANDMARKS[MY_NAME]
sensor = init_sns.sensors[MY_NAME]

XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))

vel_pub = rp.Publisher('cmd_vel', cms.Velocity, queue_size=10)
#lmks_pub = rp.Publisher('landmarks', cms.LandmarkArray, queue_size=10)
cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)

rp.wait_for_service('/draw_landmarks')
draw_landmarks_proxy = rp.ServiceProxy(
    '/draw_landmarks',
    csv.DrawLandmarks)

msg = csv.DrawLandmarksRequest(
    name = MY_NAME,
    landmarks = [lmk.to_msg() for lmk in landmarks])
draw_landmarks_proxy(msg)


#def yield_landmarks_handler(req):
#    global landmarks
#    global sensor
#    global lock
#    global PARTNERS, possible_partners
#    your_pos = np.array(req.position)
#    your_ori = np.array(req.orientation)
#    you = sn.Sensor(pos=your_pos, ori=your_ori, fp=fp.ConvexFootprint())
#    your_lmks = [lm.Landmark.from_msg(lmkmsg) for lmkmsg in req.landmarks]
#    mine = set()
#    yours = set()
#    lock.acquire()
#    for lmk in your_lmks:
#        if sensor.perception(lmk) < you.perception(lmk):
#            mine.add(lmk)
#        else:
#            yours.add(lmk)
#    if len(mine)>0:
#        possible_partners = list(PARTNERS)
#        landmarks |= mine
#    lock.release()
#    return csv.TradeLandmarksResponse(
#        success = len(mine)>0,
#        your_landmarks=[lmk.to_msg() for lmk in yours])
#
#yield_srv = rp.Service(
#    'yield_landmarks',
#    csv.TradeLandmarks,
#    yield_landmarks_handler)

#yield_proxies = dict()
#for partner in PARTNERS:
#    rp.wait_for_service('/'+partner+'/yield_landmarks')
#    yield_proxies[partner] = rp.ServiceProxy(
#        '/'+partner+'/yield_landmarks', csv.TradeLandmarks)



#def trade_landmarks_handler(req):
#    global landmarks
#    global sensor
#    global lock
#    global PARTNERS, possible_partners
#    global MY_NAME
#    your_pos = np.array(req.position)
#    your_ori = np.array(req.orientation)
#    you = sn.Sensor(pos=your_pos, ori=your_ori, fp=fp.EggFootprint())
#    your_lmks = [lm.Landmark.from_msg(lmkmsg) for lmkmsg in req.landmarks]
#    mine = set()
#    yours = set()
#    success = False
#    lock.acquire()
#    for lmk in landmarks:
#        if you.perception(lmk) < sensor.perception(lmk):
#            yours.add(lmk)
#            success = True
#        else:
#            mine.add(lmk)
#    for lmk in your_lmks:
#        if sensor.perception(lmk) < you.perception(lmk):
#            mine.add(lmk)
#            success = True
#        else:
#            yours.add(lmk)
#    if success:
#        landmarks = mine
#        possible_partners = list(PARTNERS)
#    lock.release()
#    msg = csv.DrawLandmarksRequest(
#        name = MY_NAME,
#        landmarks = [lmk.to_msg() for lmk in landmarks])
#    draw_landmarks_proxy(msg)
#    return csv.TradeLandmarksResponse(
#        success = success,
#        your_landmarks=[lmk.to_msg() for lmk in yours])
#
#trade_srv = rp.Service(
#    'trade_landmarks',
#    csv.TradeLandmarks,
#    trade_landmarks_handler)

#trade_proxies = dict()
#for partner in PARTNERS:
#    rp.wait_for_service('/'+partner+'/trade_landmarks')
#    trade_proxies[partner] = rp.ServiceProxy(
#        '/'+partner+'/trade_landmarks', csv.TradeLandmarks)



def take_landmarks_handler(req):
    global incoming_landmarks, incoming_landmarks_lock
    global sensor, sensor_lock
    client = sns.Sensor(
        pos = np.array(req.client_pose.position),
        ori = np.array(req.client_pose.orientation),
        fp = init_sns.sensors[req.client_name]._fp
        )
    client_landmarks = [
        lm.Landmark.from_msg(lmk_msg) for lmk_msg in req.client_landmarks
        ]
    my_new_landmarks = set()
    client_new_landmarks = set()
    success = False
    sensor_lock.acquire()
    for lmk in client_landmarks:
        if sensor.perception(lmk) < client.perception(lmk):
            my_new_landmarks.add(lmk)
            success = True
        else:
            client_new_landmarks.add(lmk)
    sensor_lock.release()
    if success:
        incoming_landmarks_lock.acquire()
        incoming_landmarks |= my_new_landmarks
        incoming_landmarks_lock.release()
    return csv.TakeLandmarksResponse(
        success = success,
        client_new_landmarks = [lmk.to_msg() for lmk in client_new_landmarks]
        )

take_landmarks_service = rp.Service(
    'take_landmarks',
    csv.TakeLandmarks,
    take_landmarks_handler
    )

take_landmarks_proxies = dict()
for partner in PARTNERS:
    rp.wait_for_service('/'+partner+'/take_landmarks')
    take_landmarks_proxies[partner] = rp.ServiceProxy(
        '/'+partner+'/take_landmarks',
        csv.TakeLandmarks
        )






def add_landmark_arc_handler(req):
    global incoming_landmarks, incoming_landmarks_lock
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
    incoming_landmarks_lock.acquire()
    if len(lmks)>0:
        incoming_landmarks |= lmks
    incoming_landmarks_lock.release()
    return csv.AddLandmarkArcResponse()

add_lma_srv = rp.Service(
    'add_landmark_arc',
    csv.AddLandmarkArc,
    add_landmark_arc_handler)





def add_random_landmarks_handler(req):
    global incoming_landmarks, incoming_landmarks_lock
    new_lmks = set()
    for index in range(req.num):
        new_lmks.add(lm.Landmark.random(
	        xlim=0.5*np.array(XLIM),
	        ylim=0.5*np.array(YLIM)))
    incoming_landmarks_lock.acquire()
    incoming_landmarks |= new_lmks
    incoming_landmarks_lock.release()
    return csv.AddRandomLandmarksResponse()

add_lmk_srv = rp.Service(
    'add_random_landmarks',
    csv.AddRandomLandmarks,
    add_random_landmarks_handler
)



def change_gains_handler(req):
    global kp, kn
    global sensor_lock
    sensor_lock.acquire()
    kp = req.position_gain
    kn = req.orientation_gain
    sensor_lock.release()
    return csv.ChangeGainsResponse()
chg_gns_srv = rp.Service(
    'change_gains',
    csv.ChangeGains,
    change_gains_handler)


def pose_cb(pose):
    global sensor, sensor_lock
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    sensor_lock.acquire()
    sensor.pos = p
    sensor.ori = n
    sensor_lock.release()
pose_sub = rp.Subscriber(
    'pose',
    cms.Pose,
    pose_cb)


def add_landmark_handler(req):
    global incoming_landmarks, incoming_landmarks_lock
    lmk = lm.Landmark.from_msg(req.landmark)
    incoming_landmarks_lock.acquire()
    incoming_landmarks.add(lmk)
    incoming_landmarks_lock.release()
    return csv.AddLandmarkResponse()

add_lmk_srv = rp.Service(
    'add_landmark',
    csv.AddLandmark,
    add_landmark_handler)


def change_landmarks_handler(req):
    global landmarks, landmarks_lock
    lmks = [lm.Landmark.from_message(lmk_msg) for lmk_msg in req.landmarks]
    landmarks_lock.acquire()
    landmarks = set(lmks)
    landmarks_lock.release()
    msg = csv.DrawLandmarksRequest(
        name = MY_NAME,
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



FAST_RATE = rp.Rate(6e1)
SLOW_RATE = rp.Rate(0.5)

def work():
    global sensor, sensor_lock
    global landmarks, landmarks_lock
    global incoming_landmarks, incoming_landmarks_lock
    global vel_pub#, lmks_pub
    global kp, kn
    global last_trade_time
    global possible_partners
    landmarks_lock.acquire()
    incoming_landmarks_lock.acquire()
    if len(incoming_landmarks)>0:
        landmarks |= incoming_landmarks
        incoming_landmarks = set()
        possible_partners = list(PARTNERS)
        draw_landmarks_request = csv.DrawLandmarksRequest(
            name = MY_NAME,
            landmarks = [lmk.to_msg() for lmk in landmarks]
            )
        draw_landmarks_proxy(draw_landmarks_request)
    incoming_landmarks_lock.release()
    sensor_lock.acquire()
    p = sensor.pos
    n = sensor.ori
    if len(landmarks) == 0:
        v = np.zeros(2)
        w = 0.0
    else:
        v = -kp/float(len(landmarks))*sensor.cov_pos_grad(landmarks)
        v = uts.saturate(v, sp)
        w = -kn/float(len(landmarks))*np.cross(n, sensor.cov_ori_grad(landmarks))
        w = uts.saturate(w, sn)
    coverage = sensor.coverage(landmarks)
    sensor_lock.release()
    if np.linalg.norm(v)+np.linalg.norm(w) < 2e-4*coverage:
        v = np.zeros(2)
        w = 0.0
        rp.logwarn(MY_NAME + ': possible partners: ' + str(possible_partners))
        if len(possible_partners)>0:
            request = csv.TakeLandmarksRequest(
                MY_NAME,
                cms.Pose(p,n),
                [lmk.to_msg() for lmk in landmarks]
                )
            partner = rdm.choice(possible_partners)
            rp.logwarn(MY_NAME + ': I will give landmarks to ' + partner)
            try:
                response = take_landmarks_proxies[partner](request)
                if response.success:
                    possible_partners = list(PARTNERS)
                    landmarks = set([
                        lm.Landmark.from_msg(item) for item in response.client_new_landmarks])
                    msg = csv.DrawLandmarksRequest(
                        name = MY_NAME,
                        landmarks = [lmk.to_msg() for lmk in landmarks])
                    draw_landmarks_proxy(msg)
                    rp.logwarn(MY_NAME + ': I gave some landmarks to ' + partner)
                else:
                    possible_partners.remove(partner)
                    rp.logwarn(MY_NAME + ': I could not give any landmark to ' + partner)
            except Exception as err:
                rp.logwarn(MY_NAME + ': ' + partner + ' is unavailable')
                rp.logwarn(err)
        else:
            rp.logwarn(MY_NAME + ': no possible partners')
        rate = SLOW_RATE
    else:
        possible_partners = list(PARTNERS)
        rate = FAST_RATE
    #lmks_msg = [lmk.to_msg() for lmk in landmarks]
    landmarks_lock.release()
    cov_vel = cms.Velocity(linear=v, angular=w)
    vel_pub.publish(cov_vel)
    #lmks_pub.publish(lmks_msg)
    cov_pub.publish(coverage)
    rate.sleep()








if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
