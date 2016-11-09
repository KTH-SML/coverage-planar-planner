#! /usr/bin/env python
"""This node routes:
    - Velocity messages to geometry_msgs/Twist messages;
    - geometry_msgs/Pose messages to Pose messages.
"""

import rospy as rp
import tf.transformations as tft
import threading as thd
import numpy as np
import copy as cp

import geometry_msgs.msg as gms
import trajectory_msgs.msg as tms
import coverage_planar_planner.msg as cms


rp.init_node('router_node')

twist_pub = rp.Publisher(
    'cmd_twist',
    gms.Twist,
    queue_size=10)
cmd_traj_pub = rp.Publisher(
    'cmd_traj',
    tms.MultiDOFJointTrajectory,
    queue_size=10)
pose_pub = rp.Publisher(
    'pose',
    cms.Pose,
    queue_size=10
)

pose_lock = thd.Lock()
current_pose = None

def vel_callback(msg):
    global pose_lock, current_pose
    twist = gms.Twist()
    twist.linear.x = msg.linear[0]
    twist.linear.y = msg.linear[1]
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = msg.angular
    twist_pub.publish(twist)
    cmd_traj = tms.MultiDOFJointTrajectory()
    cmd_traj.points.append(tms.MultiDOFJointTrajectoryPoint())
    pose_lock.acquire()
    ps = cp.copy(current_pose)
    pose_lock.release()
    if not ps is None:
        translation = current_pose.position
        rotation = current_pose.orientation
        transform = gms.Transform(translation, rotation)
        cmd_traj.points[0].transforms.append(transform)
        cmd_traj.points[0].velocities.append(twist)
        cmd_traj_pub.publish(cmd_traj)

def pose_callback(msg):
    global pose_lock, current_pose
    pose_lock.acquire()
    current_pose = cp.copy(msg)
    pose_lock.release()
    pose = cms.Pose()
    pose.position[0] = msg.position.x
    pose.position[1] = msg.position.y
    quaternion = np.array([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ])
    matrix = tft.quaternion_matrix(quaternion)
    pose.orientation = np.array(matrix[0:2,0])
    #pose_pub.publish(pose)



pose_sub = rp.Subscriber('gms_pose', gms.Pose, pose_callback)
vel_sub = rp.Subscriber('cmd_vel', cms.Velocity, vel_callback)
rp.spin()
