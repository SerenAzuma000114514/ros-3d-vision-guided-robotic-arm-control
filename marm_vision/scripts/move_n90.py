#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import math
from trajectory_msgs.msg import JointTrajectoryPoint

class Joint1Rotator:
    def __init__(self):
        # 初始化 moveit_commander 和 ROS 节点
        if not rospy.core.is_initialized():
            rospy.init_node('joint1_rotate_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 获取机械臂的 planning group（根据你的配置，组名通常是 "arm"）
        self.arm = moveit_commander.MoveGroupCommander("arm")
        
        # 可选：设置关节位置容差（弧度）
        self.arm.set_goal_joint_tolerance(0.01)
        
        rospy.loginfo("Joint1Rotator 初始化完成")

    def rotate_joint1(self, deg):
        """
        让 joint1 旋转指定的角度（单位：度）
        :param deg: 角度值，正负取决于关节方向（正值通常逆时针，负值顺时针）
        """
        # 获取当前所有关节的角度（弧度）
        current_joint_values = self.arm.get_current_joint_values()
        joint_names = self.arm.get_active_joints()
        
        # 找到 joint1 在关节列表中的索引
        try:
            idx = joint_names.index("joint1")
        except ValueError:
            rospy.logerr("关节列表中没有找到 joint1！")
            return False
        
        # 将目标角度从度转换为弧度，并计算新目标值
        rad = math.radians(deg)
        target_joint_values = list(current_joint_values)
        target_joint_values[idx] = current_joint_values[idx] + rad
        
        # 设置目标关节值并规划
        self.arm.set_joint_value_target(target_joint_values)
        
        rospy.loginfo(f"正在规划：将 joint1 旋转 {deg} 度（{rad:.3f} rad）...")
        success, plan, planning_time, error_code = self.arm.plan()
        
        if success:
            rospy.loginfo("规划成功，开始执行...")
            self.arm.execute(plan, wait=True)
            rospy.loginfo("运动完成")
            return True
        else:
            rospy.logwarn("规划失败，可能超出关节限位或遇到障碍物")
            self.arm.clear_pose_targets()
            return False

if __name__ == '__main__':
    try:
        rotator = Joint1Rotator()
        rospy.sleep(1.0)   # 等待节点稳定
        
        # 将 joint1 转动 -90 度（负值）
        success = rotator.rotate_joint1(-90)
        
        if not success:
            rospy.logerr("关节运动失败")
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生异常: {e}")
