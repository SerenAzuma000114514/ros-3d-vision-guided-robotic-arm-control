#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander
import tf.transformations as tf_trans
from geometry_msgs.msg import Pose, Point, Quaternion

# ===== 新增导入：用于向MoveIt添加地面碰撞物 =====
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from tf.transformations import quaternion_from_euler
# ==============================================

class ArmMover:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("arm")
        
        # ===== 新增：将地面添加为碰撞物体 =====
        rospy.loginfo("正在向规划场景添加地面...")
        self.scene = PlanningSceneInterface()
        rospy.sleep(1)  # 等待场景接口准备
        
        # 创建地面的位姿（位于 world 坐标系原点，水平面）
        pose = PoseStamped()
        pose.header.frame_id = "world"   # 如果无效，可改为 "odom" 或 "base_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        # 平面法线向上 (0,0,1)，offset=0 表示平面经过原点
        quat = quaternion_from_euler(0, 0, 0)  # 无旋转，水平面
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        self.scene.add_plane("ground_plane", pose, normal=(0, 0, 1), offset=0)
        rospy.sleep(1)
        rospy.loginfo("地面已添加，机械臂规划将自动避开地面")
        # ====================================
        
        rospy.loginfo("机械臂运动模块初始化完成")

    def get_downward_orientation(self):
        rot_mat = tf_trans.rotation_matrix(3.14159, [1, 0, 0])
        quat = tf_trans.quaternion_from_matrix(rot_mat)
        return quat

    def move_to_pose(self, target_pose):
        self.arm.set_pose_target(target_pose)
        rospy.loginfo("开始路径规划...")
        success, plan, planning_time, error_code = self.arm.plan()
        if success:
            rospy.loginfo("路径规划成功，执行中...")
            self.arm.execute(plan, wait=True)
            rospy.loginfo("已到达目标位置")
        else:
            rospy.logwarn("路径规划失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def move_to(self, x, y, z, orientation_quat=None):
        target_pose = Pose()
        target_pose.position = Point(x, y, z)
        if orientation_quat is None:
            q = self.get_downward_orientation()
            target_pose.orientation = Quaternion(*q)
        else:
            target_pose.orientation = Quaternion(*orientation_quat)
        return self.move_to_pose(target_pose)

if __name__ == '__main__':
    rospy.init_node('arm_mover_test')
    mover = ArmMover()
    rospy.sleep(1.0)

    test_x, test_y, test_z = 0.45, 0.1, 0.2
    rospy.loginfo(f"测试运动目标: ({test_x}, {test_y}, {test_z})，末端朝下")
    success = mover.move_to(test_x, test_y, test_z)
    if success:
        rospy.loginfo("运动测试成功")
    else:
        rospy.logwarn("运动测试失败")
    rospy.loginfo("程序结束")



