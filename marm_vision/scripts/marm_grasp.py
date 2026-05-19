#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os

# 强制添加scripts目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../scripts'))
# 或者直接写绝对路径
sys.path.insert(0, '/home/herunjin/gra_pj_ws/src/marm_vision/scripts')

import rospy
import moveit_commander
from sensor_msgs.msg import PointCloud2
from cloud_processor import cloud_callback, get_target_centroid
from arm_controller import init_arm, transform_to_world, move_to

rospy.init_node('main_task')
moveit_commander.roscpp_initialize([])

init_arm()
rospy.Subscriber("/camera/depth/points", PointCloud2, cloud_callback)
rospy.sleep(2.0)

centroid, frame_id = get_target_centroid()
if centroid is not None:
    pose_world = transform_to_world(centroid, frame_id)
    if pose_world is not None:
        move_to(pose_world)
