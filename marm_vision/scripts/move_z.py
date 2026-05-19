#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion

class MoveZ:
    def __init__(self):
        # 初始化 moveit_commander 和 ROS 节点（如果尚未初始化）
        if not rospy.core.is_initialized():
            rospy.init_node('move_z_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 获取机械臂的 planning group（请根据你的实际组名修改，常见为 "arm"）
        self.arm = moveit_commander.MoveGroupCommander("arm")
        
        # 可选：设置一些规划参数（可忽略或按需调整）
        self.arm.set_goal_position_tolerance(0.01)   # 位置容差 1cm
        self.arm.set_goal_orientation_tolerance(0.01) # 姿态容差 0.01 rad
        
        rospy.loginfo("MoveZ 初始化完成")

    def get_current_pose(self):
        """获取当前末端执行器的位姿"""
        return self.arm.get_current_pose().pose

    def move_z(self, delta_z):
        """
        沿 Z 轴平移指定的距离（单位：米）
        :param delta_z: 正值向上移动，负值向下移动
        """
        # 获取当前位姿
        current_pose = self.get_current_pose()
        
        # 计算目标位姿：位置改变 Z，方向不变
        target_pose = Pose()
        target_pose.position = Point(
            x = current_pose.position.x,
            y = current_pose.position.y,
            z = current_pose.position.z + delta_z
        )
        target_pose.orientation = current_pose.orientation
        
        # 设置规划目标
        self.arm.set_pose_target(target_pose)
        
        # 规划路径
        rospy.loginfo(f"正在规划沿 Z 轴平移 {delta_z:.3f} 米...")
        success, plan, planning_time, error_code = self.arm.plan()
        
        if success:
            rospy.loginfo("规划成功，开始执行...")
            # 关键：execute 需要传入 plan 对象，而不是 success 布尔值
            self.arm.execute(plan, wait=True)
            rospy.loginfo("运动完成")
        else:
            rospy.logwarn("规划失败，请检查是否遇到障碍物或超出工作空间")
        
        # 清除目标（防止下次规划残留）
        self.arm.clear_pose_targets()

if __name__ == '__main__':
    try:
        mover = MoveZ()
        rospy.sleep(1.0)  # 等待节点稳定
        
        # 示例：向上移动 0.05 米
        mover.move_z(0.05)
        
        # 如果需要向下移动，使用负值，例如：
        # mover.move_z(-0.02)
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生异常: {e}")
