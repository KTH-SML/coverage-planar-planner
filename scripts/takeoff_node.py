#! /usr/bin/env python
import rospy as rp
import geometry_msgs.msg as gms
import nav_msgs.msg as nms
import trajectory_msgs.msg as tms
import coverage_planar_planner.srv as csv

import threading as thd
import copy as cp

rp.init_node('takeoff_node')

ALTITUDE = rp.get_param('altitude')

cmd_traj_pub = rp.Publisher(
    'cmd_traj',
    tms.MultiDOFJointTrajectory,
    queue_size=10)

lock = thd.Lock()
saved_position = None
start_flag = False

def odometry_callback(msg):
    global saved_position
    lock.acquire()
    saved_position = msg.pose.pose.position
    lock.release()

rp.Subscriber(
    'odometry',
    nms.Odometry,
    odometry_callback)

def takeoff_handler(msg):
    global start_flag
    lock.acquire()
    start_flag = True
    lock.release()
    return csv.TakeoffResponse()

rp.Service(
    'takeoff',
    csv.Takeoff,
    takeoff_handler)

def work():
    global start_flag
    lock.acquire()
    if not start_flag:
        lock.release()
        return
    pos = cp.copy(saved_position)
    lock.release()
    if not pos is None:
        twist = gms.Twist()
        translation = gms.Vector3()
        translation.x = pos.x
        translation.y = pos.y
        translation.z = ALTITUDE
        rotation = gms.Quaternion()
        transform = gms.Transform(translation, rotation)
        trajectory = cmd_traj = tms.MultiDOFJointTrajectory()
        cmd_traj.points.append(tms.MultiDOFJointTrajectoryPoint())
        cmd_traj.points[0].transforms.append(transform)
        cmd_traj.points[0].velocities.append(twist)
        cmd_traj_pub.publish(cmd_traj)


RATE = rp.Rate(10)
while not rp.is_shutdown():
    work()
    RATE.sleep()
