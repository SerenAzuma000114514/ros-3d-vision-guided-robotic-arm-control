#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('gripper_release', anonymous=True)
pub = rospy.Publisher('/marm_robot/gripper_controller/command', Float64MultiArray, queue_size=10)

# 等待发布器准备好
rospy.sleep(0.5)

# 发送零力矩
msg = Float64MultiArray()
msg.data = [0.0, 0.0]
pub.publish(msg)
rospy.loginfo("夹爪已释放，力矩归零")
