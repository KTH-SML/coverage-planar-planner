#! /usr/bin/env python
"""This node routes:
    - Velocity messages to geometry_msgs/Twist messages;
    - geometry_msgs/Pose messages to Pose messages.
"""

import rospy as rp

import geometry_msgs.msg as gms
import coverage_planar_planner.msg as cms
