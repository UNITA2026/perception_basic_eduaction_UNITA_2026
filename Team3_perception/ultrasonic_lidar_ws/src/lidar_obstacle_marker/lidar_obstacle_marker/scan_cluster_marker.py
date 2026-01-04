#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def euclidean_cluster(points, tol=0.08, min_pts=3):
    """단순 2D 유클리드 클러스터링(BFS)."""
    if not points:
        return []

    n = len(points)
    visited = [False] * n
    clusters = []
    tol2 = tol * tol

    for i in range(n):
        if visited[i]:
            continue

        q = deque([i])
        visited[i] = True
        cluster = [i]

        while q:
            cur = q.popleft()
            cx, cy = points[cur]

            for j in range(n):
                if visited[j]:
                    continue
                x, y = points[j]
                dx = x - cx
                dy = y - cy
                if dx * dx + dy * dy <= tol2:
                    visited[j] = True
                    q.append(j)
                    cluster.append(j)

        if len(cluster) >= min_pts:
            clusters.append(cluster)

    return clusters


def cluster_centroid(points, idxs):
    cx = sum(points[i][0] for i in idxs) / len(idxs)
    cy = sum(points[i][1] for i in idxs) / len(idxs)
    return (cx, cy)


def merge_clusters_by_centroid(points, clusters, merge_dist=0.30):
    """
    중심 간 거리 기준으로 클러스터 병합.
    merge_dist(m): 예) 0.30이면 30cm 이내 중심은 합침
    """
    if len(clusters) <= 1:
        return clusters

    parent = list(range(len(clusters)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    centroids = [cluster_centroid(points, c) for c in clusters]
    md2 = merge_dist * merge_dist

    for i in range(len(clusters)):
        xi, yi = centroids[i]
        for j in range(i + 1, len(clusters)):
            xj, yj = centroids[j]
            dx = xi - xj
            dy = yi - yj
            if dx * dx + dy * dy <= md2:
                union(i, j)

    merged = {}
    for i, c in enumerate(clusters):
        r = find(i)
        merged.setdefault(r, []).extend(c)

    return list(merged.values())


class ScanClusterMarker(Node):
    def __init__(self):
        super().__init__('scan_cluster_marker')

        # ---- 파라미터 ----
        self.declare_parameter('in_topic', '/scan')
        self.declare_parameter('max_range', 1.0)       # ✅ 1m 이내면 1.0 (원래 값 0.15는 15cm)
        self.declare_parameter('cluster_tol', 0.2)     # 클러스터 묶는 거리(m)
        self.declare_parameter('min_pts', 3)           # 최소 점 개수
        self.declare_parameter('marker_scale', 0.12)   # 마커(구) 크기(m)
        self.declare_parameter('merge_dist', 0.30)     # 중심 30cm 이내 병합

        self.in_topic = self.get_parameter('in_topic').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.cluster_tol = float(self.get_parameter('cluster_tol').value)
        self.min_pts = int(self.get_parameter('min_pts').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.merge_dist = float(self.get_parameter('merge_dist').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(LaserScan, self.in_topic, self.cb, qos)
        self.pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self.get_logger().info(
            f"Sub: {self.in_topic} | max_range={self.max_range}m "
            f"| tol={self.cluster_tol}m | min_pts={self.min_pts} | merge_dist={self.merge_dist}m "
            f"| pub=/obstacle_markers"
        )

    def cb(self, msg: LaserScan):
        # 1) max_range 이내 점만 뽑아서 (x,y) 변환
        pts = []
        min_valid = float(msg.range_min)
        max_valid = min(float(msg.range_max), self.max_range)

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and (min_valid <= r <= max_valid):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                pts.append((x, y))
            angle += msg.angle_increment

        # 2) 클러스터링 + 중심거리 병합
        clusters = euclidean_cluster(pts, tol=self.cluster_tol, min_pts=self.min_pts)
        clusters = merge_clusters_by_centroid(pts, clusters, merge_dist=self.merge_dist)

        # 3) MarkerArray 구성
        marr = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = msg.header.frame_id
        delete_all.header.stamp = msg.header.stamp
        delete_all.action = Marker.DELETEALL
        marr.markers.append(delete_all)

        for cid, idxs in enumerate(clusters):
            cx, cy = cluster_centroid(pts, idxs)

            m = Marker()
            m.header.frame_id = msg.header.frame_id
            m.header.stamp = msg.header.stamp
            m.ns = "obstacles"
            m.id = cid
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = self.marker_scale
            m.scale.y = self.marker_scale
            m.scale.z = self.marker_scale

            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.9

            marr.markers.append(m)

        self.pub.publish(marr)


def main():
    rclpy.init()
    node = ScanClusterMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
