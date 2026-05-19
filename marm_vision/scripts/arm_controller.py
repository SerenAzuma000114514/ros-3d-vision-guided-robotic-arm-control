#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

# 全局变量
arm = None
tf_buffer = None

def init_arm():
    """初始化机械臂和TF，程序开始时调用一次"""
    global arm, tf_buffer

    # ===== 改成你的规划组名 =====
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("机械臂初始化完成")

def transform_to_world(centroid, frame_id):
    """
    把相机坐标系下的质心转换到world坐标系
    输入: centroid(长度为3的数组), frame_id(字符串)
    返回: PoseStamped 或 None
    """
    pose_cam = PoseStamped()
    pose_cam.header.frame_id = frame_id
    pose_cam.header.stamp = rospy.Time(0)
    pose_cam.pose.position.x = float(centroid[0])
    pose_cam.pose.position.y = float(centroid[1])
    pose_cam.pose.position.z = float(centroid[2])
    pose_cam.pose.orientation.w = 1.0

    try:
        pose_world = tf_buffer.transform(pose_cam, "world", rospy.Duration(1.0))
        rospy.loginfo("目标位置(world系): x=%.3f y=%.3f z=%.3f" % (
            pose_world.pose.position.x,
            pose_world.pose.position.y,
            pose_world.pose.position.z))
        return pose_world
    except Exception as e:
        rospy.logwarn("TF转换失败: %s" % str(e))
        return None

def move_to(pose_world):
    """
    让机械臂移动到目标位置上方
    输入: PoseStamped(world系)
    返回: True/False
    """
    # ===== 末端悬停高度，单位米 =====
    pose_world.pose.position.z += 0.1

    arm.set_pose_target(pose_world)
    success, plan, planning_time, error_code = arm.plan()

    if success:
        rospy.loginfo("规划成功，开始运动...")
        arm.execute(plan, wait=True)
        arm.stop()
        arm.clear_pose_targets()
        rospy.loginfo("运动完成")
        return True
    else:
        rospy.logwarn("规划失败")
        arm.stop()
        arm.clear_pose_targets()
        return False


# 只在直接运行时才执行，import时不执行
if __name__ == '__main__':
    rospy.init_node('arm_controller_test')
    moveit_commander.roscpp_initialize([])

    init_arm()
    rospy.loginfo("测试：让机械臂回到初始位置")
    arm.set_named_target("home")  # 需要在MoveIt中定义home位姿
    arm.go(wait=True)
