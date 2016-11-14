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

rp.init_node('lee_controller_bridge')
ALTITUDE = rp.get_param('altitude')

cmd_traj_pub = rp.Publisher(
    'cmd_traj',
    tms.MultiDOFJointTrajectory,
    queue_size=10)


pose_lock = thd.Lock()
saved_pose = None


def cmd_vel_callback(msg):
    global pose_lock, saved_pose
    twist = gms.Twist()
    twist.linear.x = msg.linear[0]
    twist.linear.y = msg.linear[1]
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = msg.angular
    pose_lock.acquire()
    ps = cp.copy(saved_pose)
    pose_lock.release()
    if not ps is None:
        translation = gms.Vector3()
        translation.x = ps.position[0]
        translation.y = ps.position[1]
        translation.z = ALTITUDE
        rot_mat = np.eye(4)
        rot_mat[0:2,0] = ps.orientation
        rot_mat[0:2,1] = utl.ccws_perp(ps.orientation)
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
