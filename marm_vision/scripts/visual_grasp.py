#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans

class VisualGrasp:
    def __init__(self):
        rospy.init_node('visual_grasp')
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.triggered = False
        # 发布聚类包围盒可视化
        self.bbox_pub = rospy.Publisher("/object_bbox", Marker, queue_size=10)
        # 末端执行器link名称：使用机械臂的最后一个连杆 Link6（夹爪基座固定在其上）
        self.ee_link = "Link6"
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.cloud_callback)
        rospy.loginfo("等待点云...")

    def apply_clustering(self, points, eps=0.03, min_points=20):
        if len(points) < min_points:
            return None, None
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        max_label = labels.max()
        if max_label < 0:
            return None, None
        cluster_counts = np.bincount(labels[labels >= 0])
        largest_label = cluster_counts.argmax()
        largest_points = points[labels == largest_label]
        return largest_points, labels

    def transform_points_to_world(self, points_cam, header_frame_id, header_stamp):
        """将相机坐标系下的点云批量转换到 world 坐标系"""
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
        """从世界坐标系点云中计算顶部中心"""
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

    def publish_bbox(self, points_world, header_stamp):
        """发布点云在世界坐标系下的AABB包围盒"""
        if len(points_world) == 0:
            return
        min_pt = np.min(points_world, axis=0)
        max_pt = np.max(points_world, axis=0)
        center = (min_pt + max_pt) / 2
        size = max_pt - min_pt
        
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = header_stamp
        marker.ns = "object_bbox"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0.5)
        self.bbox_pub.publish(marker)

    def compute_min_distance_to_cluster(self, world_points, header_stamp):
        """
        计算末端执行器（Link6）到点云簇的最短欧氏距离（世界坐标系）
        返回距离值，如果失败返回 None
        """
        if world_points is None or len(world_points) == 0:
            return None
        try:
            trans = self.tf_buffer.lookup_transform("world", self.ee_link, header_stamp, rospy.Duration(0.5))
            ee_pos = trans.transform.translation
            ee_point = np.array([ee_pos.x, ee_pos.y, ee_pos.z])
        except Exception as e:
            rospy.logerr("获取末端执行器位姿失败: %s", e)
            return None

        distances = np.linalg.norm(world_points - ee_point, axis=1)
        min_dist = np.min(distances)
        rospy.loginfo("末端执行器 (%s) 到点云簇的最短距离: %.3f m", self.ee_link, min_dist)
        return min_dist

    def cloud_callback(self, msg):
        if self.triggered:
            return

        # 1. 读取点云
        points = np.array([[p[0], p[1], p[2]] for p in
                           pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)])
        rospy.loginfo(f"原始点云: {len(points)}")
        if len(points) == 0:
            return

        # 2. 去除桌面
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                 ransac_n=3, num_iterations=100)
        pcd_objects = pcd.select_by_index(inliers, invert=True)
        object_points = np.asarray(pcd_objects.points)
        rospy.loginfo(f"去除桌面后: {len(object_points)}")

        # 3. 去除地面
        if len(object_points) > 0:
            pcd_temp = o3d.geometry.PointCloud()
            pcd_temp.points = o3d.utility.Vector3dVector(object_points)
            ground_plane, ground_inliers = pcd_temp.segment_plane(distance_threshold=0.02,
                                                                  ransac_n=3, num_iterations=100)
            a, b, c, d = ground_plane
            if abs(c) > 1e-3:
                plane_z = -d / c
            else:
                plane_z = 0.0
            if c > 0.9 and plane_z < 0.2:
                pcd_no_ground = pcd_temp.select_by_index(ground_inliers, invert=True)
                object_points = np.asarray(pcd_no_ground.points)
                rospy.loginfo(f"去除地面后（高度 {plane_z:.3f} m）: {len(object_points)}")
                self.ground_z = plane_z
            else:
                self.ground_z = 0.0
        else:
            self.ground_z = 0.0

        if len(object_points) < 50:
            rospy.loginfo("去除桌面/地面后点数过少，跳过")
            return

        # 4. 聚类并提取最大物体点云
        largest_points, _ = self.apply_clustering(object_points, eps=0.03, min_points=20)
        if largest_points is None or len(largest_points) < 20:
            rospy.logwarn("未提取到有效物体")
            return
        rospy.loginfo(f"最大物体点数: {len(largest_points)}")

        # 5. 将物体点云转换到世界坐标系
        world_pts = self.transform_points_to_world(largest_points,
                                                   msg.header.frame_id,
                                                   msg.header.stamp)
        if len(world_pts) < 10:
            rospy.logwarn("转换后世界点云太少")
            return

        # 6. 可视化物体包围盒
        self.publish_bbox(world_pts, msg.header.stamp)

        # ========== 7. 计算末端（Link6）到点云簇的最短距离 ==========
        min_dist = self.compute_min_distance_to_cluster(world_pts, msg.header.stamp)
        if min_dist is None:
            rospy.logwarn("无法计算距离，跳过本次规划")
            return

        # 如果距离过小（小于 0.03 m = 3 cm），则认为已经太近，不再移动
        if min_dist < 0.03:
            rospy.logwarn("末端已经非常接近物体（距离 %.3f m），为避免碰撞，跳过运动", min_dist)
            self.arm.stop()
            self.arm.clear_pose_targets()
            self.triggered = True   # 任务完成，不再处理后续点云
            return

        # 8. 计算顶部中心（世界坐标系）
        top_center = self.compute_top_center_world(world_pts, top_ratio=0.2)
        if top_center is None:
            return

        # 9. 目标点 = 顶部中心 + 抬高
        target_x = top_center[0]
        target_y = top_center[1]
        target_z = top_center[2] + 0.05

        # 地面碰撞限制
        ground_limit = getattr(self, 'ground_z', 0.0) + 0.1
        if target_z < ground_limit:
            rospy.logwarn(f"目标Z坐标 {target_z:.3f} 低于地面限制 {ground_limit:.3f}，禁止运动")
            self.arm.stop()
            self.arm.clear_pose_targets()
            self.triggered = False
            return

        # 构造目标位姿（末端垂直朝下）
        q = tf_trans.quaternion_from_matrix(tf_trans.rotation_matrix(np.pi, [1, 0, 0]))
        target_pose = Pose()
        target_pose.position = Point(target_x, target_y, target_z)
        target_pose.orientation = Quaternion(*q)

        rospy.loginfo(f"最终目标 world: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")

        # 10. 运动规划
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
    try:
        vs = VisualGrasp()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
