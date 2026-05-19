#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans

class VisualGrasp:
    def __init__(self):
        rospy.init_node('visual_grasp')
        
        # 初始化机械臂控制组
        self.arm = moveit_commander.MoveGroupCommander("arm") 
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.triggered = False
        
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.cloud_callback)
        rospy.loginfo("视觉抓取节点已启动，等待点云...")

    def cloud_callback(self, msg):
        if self.triggered:
            return
        
        # 1. 读取点云并转换为numpy阵列
        points = np.array([[p[0], p[1], p[2]] for p in
                           pc2.read_points(msg, field_names=("x","y","z"),
                           skip_nans=True)], dtype=np.float32)
        
        if len(points) < 10:
            rospy.logwarn("点云数据不足")
            return

        rospy.loginfo(f"原始点云数量: {len(points)} 个点")
        
        # 2. 简单高度滤波：删除低于地面的点
        # 假设地面在相机坐标系下的z=0.3米处（根据实际情况调整）
        ground_height = 0.3
        mask = points[:, 2] > ground_height
        filtered_points = points[mask]
        
        if len(filtered_points) < 10:
            rospy.logwarn("高度滤波后点云数量不足")
            return
        
        rospy.loginfo(f"高度滤波后点云数量: {len(filtered_points)} 个点")
        
        # 3. 计算点云重心
        centroid = filtered_points.mean(axis=0)
        rospy.loginfo(f"物体重心 (相机系): [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")

        # 4. 构造 PoseStamped
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = msg.header.frame_id
        pose_cam.header.stamp = rospy.Time(0)
        pose_cam.pose.position.x = float(centroid[0])
        pose_cam.pose.position.y = float(centroid[1])
        pose_cam.pose.position.z = float(centroid[2])
        pose_cam.pose.orientation.w = 1.0

        # 5. TF 坐标转换 (从相机系转到世界系)
        try:
            pose_world = self.tf_buffer.transform(pose_cam, "world", rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn("TF转换失败: %s" % str(e))
            return

        # 对 world 坐标进行保留两位小数处理
        world_x = round(pose_world.pose.position.x, 2)
        world_y = round(pose_world.pose.position.y, 2)
        world_z = 0.16

        rospy.loginfo("========================================")
        rospy.loginfo("检测到物体在 [world] 坐标系的位置 (已保留两位小数):")
        rospy.loginfo("X: %.2f, Y: %.2f, Z: %.2f" % (world_x, world_y, world_z))
        
        # 6. 控制机械臂移动到检测到的位置
        success = self.move_to_world_position(world_x, world_y, world_z)
        
        if success:
            rospy.loginfo("抓取任务完成")
        else:
            rospy.logwarn("抓取任务失败")
        
        # 停止并清理
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.triggered = True
    
    def get_downward_orientation(self):
        """返回夹爪垂直朝下的四元数"""
        rotation_matrix = tf_trans.rotation_matrix(3.14159, [1, 0, 0])
        quaternion = tf_trans.quaternion_from_matrix(rotation_matrix)
        return quaternion
    
    def check_arrival(self, target_x, target_y, target_z, distance_tolerance=0.01):
        """
        检查当前末端位姿与目标位置的偏差是否在阈值内
        """
        # 获取当前的末端位姿
        current_pose = self.arm.get_current_pose().pose
        
        # 计算欧几里得距离
        dist = np.sqrt(
            (current_pose.position.x - target_x)**2 +
            (current_pose.position.y - target_y)**2 +
            (current_pose.position.z - target_z)**2
        )
        
        rospy.loginfo(f"当前末端偏差距离: {dist:.4f} 米")
        return dist < distance_tolerance
    
    def move_to_world_position(self, x, y, z, min_z_height=0.015, distance_tolerance=0.01):
        """运动到指定的World坐标位置"""
        # 高度安全检查
        if z < min_z_height:
            rospy.logwarn(f"高度安全限制: 调整 {z:.3f} -> {min_z_height}")
            z = 0.16
        
        # 获取夹爪朝下的四元数
        downward_quat = self.get_downward_orientation()
        
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position = Point(x, y, z)
        target_pose.orientation = Quaternion(*downward_quat)
        
        # 设置目标
        self.arm.set_pose_target(target_pose)
        
        rospy.loginfo(f"目标坐标: ({x}, {y}, {z})，规划中...")
        
        # 执行运动
        success = self.arm.go(wait=True)
        
        # 清理目标
        self.arm.clear_pose_targets()

        # 逻辑判定：如果运动成功或实际距离很近，判定为成功
        is_arrived = self.check_arrival(x, y, z, distance_tolerance)
        
        if success or is_arrived:
            rospy.loginfo("判定结果: 运动成功 (已进入误差容限范围)")
            return True
        else:
            rospy.logwarn("判定结果: 运动失败 (偏差过大)")
            return False

if __name__ == '__main__':
    try:
        vs = VisualGrasp()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
