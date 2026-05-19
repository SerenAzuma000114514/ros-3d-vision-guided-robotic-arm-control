#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans
from scipy.spatial import cKDTree
import struct

class VisualGrasp:
    def __init__(self):
        rospy.init_node('visual_grasp')
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.triggered = False

        # ========== 可调参数：空间裁剪区域 (相机坐标系) ==========
        self.crop_x_min = -0.6   # 左边界 (米)
        self.crop_x_max = 0.6    # 右边界
        self.crop_y_min = -0.4   # 前边界 (Y)
        self.crop_y_max = 0.5
        self.crop_z_min = 0.0    # 桌面高度附近
        self.crop_z_max = 0.8    # 最高物体高度

        # ========== KD-Tree 聚类参数 ==========
        self.cluster_tolerance = 0.03   # 欧式距离阈值 (米)
        self.min_cluster_size = 20      # 最小点数
        self.max_cluster_size = 5000    # 最大点数

        # ========== 可视化发布者 ==========
        # 裁剪后的点云 (相机坐标系)
        self.cropped_pub = rospy.Publisher("/visual_grasp/cropped_cloud", PointCloud2, queue_size=1)
        # 所有聚类点云 (每个聚类单独话题，用于 RViz 分别着色)
        self.cluster_pubs = {}
        # 最大物体点云 (相机坐标系，便于调试)
        self.largest_cluster_pub = rospy.Publisher("/visual_grasp/largest_cluster", PointCloud2, queue_size=1)

        rospy.Subscriber("/camera/depth/points", PointCloud2, self.cloud_callback)
        rospy.loginfo("等待点云...")

    # ------------------------------------------------------------
    # 点云处理辅助函数
    # ------------------------------------------------------------
    def crop_point_cloud(self, points):
        """空间裁剪：保留在长方体区域内的点"""
        mask = (points[:, 0] >= self.crop_x_min) & (points[:, 0] <= self.crop_x_max) & \
               (points[:, 1] >= self.crop_y_min) & (points[:, 1] <= self.crop_y_max) & \
               (points[:, 2] >= self.crop_z_min) & (points[:, 2] <= self.crop_z_max)
        return points[mask]

    def euclidean_clustering_kdtree(self, points, eps, min_points, max_points):
        """
        使用 scipy.cKDTree 进行欧式聚类
        返回: list of indices for each cluster
        """
        if len(points) < min_points:
            return []
        tree = cKDTree(points)
        visited = set()
        clusters = []
        for i in range(len(points)):
            if i in visited:
                continue
            # BFS 查找邻域
            queue = [i]
            cluster = []
            while queue:
                idx = queue.pop()
                if idx in visited:
                    continue
                visited.add(idx)
                cluster.append(idx)
                # 查询半径内所有邻居
                neighbors = tree.query_ball_point(points[idx], eps)
                for nb in neighbors:
                    if nb not in visited and nb not in queue:
                        queue.append(nb)
            if min_points <= len(cluster) <= max_points:
                clusters.append(cluster)
        return clusters

    def points_to_ros_cloud(self, points, frame_id, stamp):
        """将 numpy 点云数组 (Nx3) 转换为 ROS PointCloud2 消息"""
        if points is None or len(points) == 0:
            return None
        header = rospy.Header(frame_id=frame_id, stamp=stamp)
        # 构造 point_cloud2 需要的字段
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        cloud_msg = pc2.create_cloud(header, fields, points)
        return cloud_msg

    def transform_points_to_world(self, points_cam, header_frame_id, header_stamp):
        """将相机坐标系下的点云批量转换到 world 坐标系（原函数保留）"""
        world_points = []
        for pt in points_cam:
            ps = PointStamped()
            ps.header.frame_id = header_frame_id
            ps.header.stamp = header_stamp
            ps.point.x = pt[0]
            ps.point.y = pt[1]
            ps.point.z = pt[2]
            try:
                ps_world = self.tf_buffer.transform(ps, "world", rospy.Duration(0.5))
                world_points.append([ps_world.point.x, ps_world.point.y, ps_world.point.z])
            except:
                continue
        return np.array(world_points)

    def compute_top_center_world(self, world_points, top_ratio=0.2):
        """计算世界坐标系中物体的顶部中心（原函数保留）"""
        if len(world_points) == 0:
            return None
        z_vals = world_points[:, 2]
        z_threshold = np.percentile(z_vals, 100 * (1 - top_ratio))
        top_points = world_points[z_vals >= z_threshold]
        if len(top_points) == 0:
            return world_points.mean(axis=0)
        center = top_points.mean(axis=0)
        rospy.loginfo(f"顶部中心：取自 {len(top_points)}/{len(world_points)} 点，Z阈值={z_threshold:.3f}, 中心=({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")
        return center

    # ------------------------------------------------------------
    # 主回调函数：改进的点云处理流程
    # ------------------------------------------------------------
    def cloud_callback(self, msg):
        if self.triggered:
            return

        # 1. 读取原始点云 (相机坐标系)
        points = np.array([[p[0], p[1], p[2]] for p in
                           pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)])
        rospy.loginfo(f"原始点云: {len(points)}")
        if len(points) == 0:
            return

        # 2. 去除桌面平面 (RANSAC)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                 ransac_n=3, num_iterations=100)
        pcd_objects = pcd.select_by_index(inliers, invert=True)
        object_points = np.asarray(pcd_objects.points)
        rospy.loginfo(f"去除桌面后: {len(object_points)}")
        if len(object_points) < 50:
            return

        # 3. 空间裁剪 (只保留感兴趣区域)
        cropped_points = self.crop_point_cloud(object_points)
        rospy.loginfo(f"空间裁剪后: {len(cropped_points)}")
        if len(cropped_points) < 50:
            return

        # 发布裁剪后的点云（用于 RViz 调试）
        cropped_cloud_msg = self.points_to_ros_cloud(cropped_points, msg.header.frame_id, msg.header.stamp)
        if cropped_cloud_msg is not None:
            self.cropped_pub.publish(cropped_cloud_msg)

        # 4. KD-Tree 聚类 (欧式聚类)
        clusters_idx = self.euclidean_clustering_kdtree(cropped_points,
                                                        eps=self.cluster_tolerance,
                                                        min_points=self.min_cluster_size,
                                                        max_points=self.max_cluster_size)
        rospy.loginfo(f"聚类个数: {len(clusters_idx)}")
        if len(clusters_idx) == 0:
            return

        # 5. 找出点数最多的聚类（最大物体）
        largest_cluster_idx = max(clusters_idx, key=len)
        largest_points = cropped_points[largest_cluster_idx]

        # 发布所有聚类点云（每个聚类单独话题，方便 RViz 着色）
        for i, idx_list in enumerate(clusters_idx):
            cluster_pts = cropped_points[idx_list]
            topic_name = f"/visual_grasp/cluster_{i}"
            if topic_name not in self.cluster_pubs:
                self.cluster_pubs[topic_name] = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
            cluster_msg = self.points_to_ros_cloud(cluster_pts, msg.header.frame_id, msg.header.stamp)
            if cluster_msg is not None:
                self.cluster_pubs[topic_name].publish(cluster_msg)

        # 发布最大物体点云（单独主题）
        largest_msg = self.points_to_ros_cloud(largest_points, msg.header.frame_id, msg.header.stamp)
        if largest_msg is not None:
            self.largest_cluster_pub.publish(largest_msg)

        # 6. 将最大物体点云转换到世界坐标系（用于运动规划）
        world_pts = self.transform_points_to_world(largest_points,
                                                   msg.header.frame_id,
                                                   msg.header.stamp)
        if len(world_pts) < 10:
            rospy.logwarn("转换后世界点云太少")
            return

        # 7. 计算顶部中心（世界坐标系）
        top_center = self.compute_top_center_world(world_pts, top_ratio=0.2)
        if top_center is None:
            return

        # 8. 目标点 = 顶部中心 + 抬高5cm
        target_x = top_center[0]
        target_y = top_center[1]
        target_z = top_center[2] + 0.05

        q = tf_trans.quaternion_from_matrix(tf_trans.rotation_matrix(np.pi, [1, 0, 0]))
        target_pose = Pose()
        target_pose.position = Point(target_x, target_y, target_z)
        target_pose.orientation = Quaternion(*q)

        rospy.loginfo(f"最终目标 world: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")

        # 9. 运动规划（保持不变）
        self.arm.set_pose_target(target_pose)
        success, plan, _, _ = self.arm.plan()
        if success:
            rospy.loginfo("规划成功，执行中...")
            self.arm.execute(plan, wait=True)
            rospy.loginfo("到达目标位置")
        else:
            rospy.logwarn("规划失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.triggered = True

if __name__ == '__main__':
    vs = VisualGrasp()
    rospy.spin()
