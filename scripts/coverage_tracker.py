#! /usr/bin/env python

import rospy as rp
import std_msgs.msg as sms

import threading as thd

rp.init_node('coverage_tracker_node')
NAMES = rp.get_param('names').split()

coverages = dict()
lock = thd.Lock()


def callback(msg, name):
    global lock
    global coverages
    lock.acquire()
    coverages[name] = msg.data
    lock.release()


for name in NAMES:
    rp.Subscriber(
        name+'/coverage',
        sms.Float64,
        callback,
        callback_args=name)
    

pub = rp.Publisher(
    'total_coverage',
    sms.Float64,
    queue_size=10)

def work():
    global lock
    global coverages
    lock.acquire()
    total = sum(coverages.values())
    lock.release()
    pub.publish(sms.Float64(total))
    
    
    
rate = rp.Rate(1e1)
if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
        rate.sleep()
