#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray

class GripperGraspController:
    def __init__(self):
        rospy.init_node('gripper_grasp_controller', anonymous=True)
        
        # 话题名（根据你的实际命名空间）
        self.topic_name = '/marm_robot/gripper_controller/command'
        self.pub = rospy.Publisher(self.topic_name, Float64MultiArray, queue_size=10)
        
        # 控制频率 (50 Hz)
        self.rate = rospy.Rate(50)
        
        # ========== 关键修正点 ==========
        # 请根据实际测试，填写使夹爪闭合的力矩值（单位：Nm）
        # 方法：先手动发送不同符号组合，观察夹爪是否向中心闭合
        self.close_effort_left = -20.0   # 左手指闭合所需的力矩（例如 -1.0）
        self.close_effort_right = -20.0  # 右手指闭合所需的力矩（原来这里是 +1.0，现在改为相同符号）
        # ================================
        
        # 预夹紧时的比例（例如 30% 的闭合力度）
        self.pre_grasp_ratio = 0.3
        
    def send_effort(self, left, right):
        msg = Float64MultiArray()
        msg.data = [left, right]
        self.pub.publish(msg)
        
    def start_grasping(self, duration=None):
        """
        持续施加闭合力矩
        :param duration: 持续时间(秒)，None表示无限
        """
        rospy.loginfo("开始夹持，力矩: left=%.2f, right=%.2f", 
                      self.close_effort_left, self.close_effort_right)
        
        if duration is None:
            while not rospy.is_shutdown():
                self.send_effort(self.close_effort_left, self.close_effort_right)
                self.rate.sleep()
        else:
            end_time = rospy.Time.now() + rospy.Duration(duration)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                self.send_effort(self.close_effort_left, self.close_effort_right)
                self.rate.sleep()
            self.release()
            
    def release(self):
        rospy.loginfo("释放夹爪，力矩归零")
        self.send_effort(0.0, 0.0)
        
    def grasp_with_hysteresis(self, pre_grasp_time=0.5, hold_time=5.0):
        """
        带预夹紧的抓取序列
        :param pre_grasp_time: 预夹紧时间(秒)
        :param hold_time: 全力夹持保持时间(秒)
        """
        # 步骤1：预夹紧（较小力矩）
        pre_left = self.close_effort_left * self.pre_grasp_ratio
        pre_right = self.close_effort_right * self.pre_grasp_ratio
        rospy.loginfo("预夹紧阶段 (%.2f Nm, %.2f Nm)..." % (pre_left, pre_right))
        for _ in range(int(pre_grasp_time * 50)):
            self.send_effort(pre_left, pre_right)
            self.rate.sleep()
        
        # 步骤2：全力夹持
        rospy.loginfo("全力夹持阶段...")
        for _ in range(int(hold_time * 50)):
            self.send_effort(self.close_effort_left, self.close_effort_right)
            self.rate.sleep()
        
        rospy.loginfo("抓取序列完成，夹爪保持施力状态")
        # 注意：此处不自动释放，保持夹持
        
if __name__ == '__main__':
    try:
        controller = GripperGraspController()
        rospy.sleep(1)   # 等待Gazebo连接稳定
        
        # 推荐使用带预夹紧的序列
        controller.grasp_with_hysteresis(pre_grasp_time=0.5, hold_time=5.0)
        
        # 保持节点运行，持续施加夹持力
        rospy.loginfo("节点持续运行，夹爪保持闭合力矩... 按 Ctrl+C 退出")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
