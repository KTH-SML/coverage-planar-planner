#! /usr/bin/env python
import rospy as rp
import tf.transformations as tft
import threading as thd
import numpy as np
import copy as cp

import geometry_msgs.msg as gms
import nav_msgs.msg as nms
import trajectory_msgs.msg as tms
import coverage_planar_planner.msg as cms

import utilities as utl

rp.init_node('cmd_vel_to_cmd_traj_bridge')
ALTITUDE = rp.get_param('altitude')


XLIM = [float(elem) for elem in rp.get_param('xlim', "-0.4 2.4").split()]
YLIM = [float(elem) for elem in rp.get_param('ylim', "-2.0 1.4").split()]


cmd_traj_pub = rp.Publisher(
    'cmd_traj',
    tms.MultiDOFJointTrajectory,
    queue_size=10)


pose_lock = thd.Lock()
saved_pose = None
last_time = None
new_time = None

def cmd_vel_callback(msg):
    global pose_lock, saved_pose, last_time, new_time
    pose_lock.acquire()
    ps = cp.copy(saved_pose)
    pose_lock.release()
    if last_time is None:
        last_time = rp.get_time()
    else:
        last_time = new_time
    new_time = rp.get_time()
    time_step = new_time - rp.get_time()
    if ps is None:
        return
    twist = gms.Twist()
    translation = gms.Vector3()
    # if ps.position[0] <= XLIM[0] and msg.linear[0] <= 0.0:
    #     twist.linear.x = 0.0
    #     translation.x = XLIM[0]
    # elif ps.position[0] >= XLIM[1] and msg.linear[0] >= 0.0:
    #     twist.linear.x = 0.0
    #     translation.x = XLIM[1]
    # else:
    #     twist.linear.x = msg.linear[0]
    #     translation.x = ps.position[0]
    # if ps.position[1] <= YLIM[0] and msg.linear[1] <= 0.0:
    #     twist.linear.y = 0.0
    #     translation.y = YLIM[0]
    # elif ps.position[1] >= YLIM[1] and msg.linear[1] >= 0.0:
    #     twist.linear.y = 0.0
    #     translation.y = YLIM[1]
    # else:
    #     twist.linear.y = msg.linear[1]
    #     translation.y = ps.position[1]
    twist.linear.x = msg.linear[0]
    translation.x = ps.position[0]
    twist.linear.y = msg.linear[1]
    translation.y = ps.position[1]
    twist.linear.z = 0.0
    twist.angular.z = msg.angular
    translation.z = ALTITUDE
    rot_mat = np.eye(4)
    rot_mat[0:2,0] = np.array(ps.orientation) + msg.angular*utl.ccws_perp(ps.orientation)*time_step
    rot_mat[0:2,1] = utl.ccws_perp(ps.orientation) - msg.angular*np.array(ps.orientation)*time_step
    quat = tft.quaternion_from_matrix(rot_mat)
    rotation = gms.Quaternion()
    rotation.x = quat[0]
    rotation.y = quat[1]
    rotation.z = quat[2]
    rotation.w = quat[3]
    transform = gms.Transform(translation, rotation)
    cmd_traj = tms.MultiDOFJointTrajectory()
    cmd_traj.points.append(tms.MultiDOFJointTrajectoryPoint())
    cmd_traj.points[0].transforms.append(transform)
    cmd_traj.points[0].velocities.append(twist)
    cmd_traj_pub.publish(cmd_traj)


def pose_callback(msg):
    global pose_lock, saved_pose
    pose_lock.acquire()
    saved_pose = msg
    pose_lock.release()



pose_sub = rp.Subscriber('pose', cms.Pose, pose_callback)
vel_sub = rp.Subscriber('cmd_vel', cms.Velocity, cmd_vel_callback)

rp.spin()
