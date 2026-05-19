#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_minimal_cr5.py
针对CR5机器人的最小化MoveIt控制程序
"""

import rospy
import sys
import moveit_commander

# 初始化
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_marm')

# 连接到规划组 - 使用 cr5_arm
arm = moveit_commander.MoveGroupCommander('arm')

# 移动到home位置
arm.set_named_target('home')
arm.go()

# 关闭
moveit_commander.roscpp_shutdown()
