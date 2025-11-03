import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import heapq
import numpy as np
from datetime import datetime


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        # Subscribers
        self.create_subscription(NavSatFix, '/gps_raw', self.gps_callback, 10)
        self.create_subscription(NavSatFix, 'nav_goal', self.goal_callback, 10)

        # Load precomputed graph
        self.points = np.load("points.npy")
        self.neighbors = np.load("neighbors.npy", allow_pickle=True)

        self.current_pose = None
        self.get_logger().info("✅ GlobalPlanner started. Listening for nav_goal.")

    def gps_callback(self, msg: NavSatFix):
        """Update robot's current GPS position."""
        self.current_pose = (msg.latitude, msg.longitude)

    def haversine(self, lat1, lon1, lat2, lon2):
        """Compute great-circle distance in meters between two GPS points."""
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def find_nearest_node(self, lat, lon):
        """Find closest node index to given GPS coordinate."""
        min_d = float('inf')
        nearest = 0
        for i, (nlat, nlon) in enumerate(self.points):
            d = self.haversine(lat, lon, nlat, nlon)
            if d < min_d:
                min_d = d
                nearest = i
        return nearest

    def plan_path(self, start_idx, goal_idx):
        """Dijkstra search over neighbor list graph."""
        pq = [(0, start_idx, [])]
        visited = set()
        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]
            if node == goal_idx:
                return path
            for nb, dist in self.neighbors[node]:
                if nb not in visited:
                    heapq.heappush(pq, (cost + dist, nb, path))
        return []

    def publish_path(self, path_ids):
        """
        Publish the path as a Path message (lon→x, lat→y)
        and save the GPS waypoints into a timestamped npy file.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"

        gps_waypoints = []

        for pid in path_ids:
            lat, lon = self.points[pid]

            # Fill PoseStamped with lon as x, lat as y
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(lat)
            pose.pose.position.y = float(lon)
            path_msg.poses.append(pose)

            gps_waypoints.append([lat, lon])

        # Publish path
        self.path_pub.publish(path_msg)

        # Save GPS waypoints to .npy file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        filename = f"path_{timestamp}.npy"
        np.save(filename, np.array(gps_waypoints, dtype=float))

        self.get_logger().info(f"✅ Path published and saved to {filename}")

    def goal_callback(self, msg: NavSatFix):
        """Handle new goal: plan path and publish it."""
        if self.current_pose is None:
            self.get_logger().warn("⚠️ No current GPS yet, ignoring goal.")
            return

        start_idx = self.find_nearest_node(self.current_pose[0], self.current_pose[1])
        goal_idx = self.find_nearest_node(msg.latitude, msg.longitude)

        self.get_logger().info(f"Planning path from {start_idx} → {goal_idx}")

        path = self.plan_path(start_idx, goal_idx)
        if not path:
            self.get_logger().warn(f"⚠️ No path found to goal ({msg.latitude}, {msg.longitude})")
            return

        self.publish_path(path)


def main():
    rclpy.init()
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
