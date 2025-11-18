#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, Bool
import math
from collections import deque
import time
import os

DIST_THRESHOLD = 14.0
MAX_DIST_THRESHOLD = 24.0
from pino_msgs.msg import AudioMSG   # adjust package name if different

def haversine_distance(lat1, lon1, lat2, lon2):
    """Compute great-circle distance in meters between two GPS points."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class FarPlanner(Node):
    def __init__(self):
        super().__init__('far_planner')

        # Internal states
        self.robot_lat = None
        self.robot_lon = None
        self.robot_yaw = 0.0  # from odometry

        self.last_to_cur_goal = 100
        self.last_gps_time = None
        self.last_gps_lat = None
        self.last_gps_lon = None
        self.gps_stable = False

        # Queue of GPS goals (lat, lon)
        self.goal_queue = deque()

        # Subscribers
        self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.create_subscription(NavSatFix, 'gps_raw', self.gps_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Int32, "behavior_mode", self.behavior_callback, 10)

        # Publishers
        self.waypoint_pub = self.create_publisher(PointStamped, 'way_point', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        # Timer
        self.create_timer(0.25, self.timer_callback)  # 1 Hz (change to 0.2 for 5 Hz)

        # Setup log file
        log_dir = os.path.expanduser("~/javis_ws/far_planner_logs")
        os.makedirs(log_dir, exist_ok=True)
        self.waypoint_file = open(os.path.join(log_dir, "waypoints_log.txt"), "a")

        self.get_logger().info("✅ FarPlanner node started (logging to ~/far_planner_logs/waypoints_log.txt).")

    def behavior_callback(self, msg: Int32):
        behavior_mode = msg.data
        if behavior_mode != 2:
            self.goal_queue.clear()

    def path_callback(self, msg: Path):
        """Receive a Path message from global planner and fill the goal queue with GPS points."""
        self.goal_queue.clear()
        for pose in msg.poses:
            lat = pose.pose.position.x  # stored as lat
            lon = pose.pose.position.y  # stored as lon
            self.goal_queue.append((lat, lon))
        if self.goal_queue:
            self.get_logger().info(f"📥 Received path with {len(self.goal_queue)} waypoints.")

    def gps_callback(self, msg: NavSatFix):
        self.robot_lat = msg.latitude
        self.robot_lon = msg.longitude
        try:
            lat, lon = msg.latitude, msg.longitude
            now = time.time()

            if self.last_gps_time is not None:
                dt = now - self.last_gps_time
                freq = 1.0 / dt if dt > 0 else 0.0
                dist = haversine_distance(self.last_gps_lat, self.last_gps_lon, lat, lon)

                if freq > 0.25 and dist < 4.0:
                    self.gps_stable = True
                else:
                    self.gps_stable = False
                    self.get_logger().warn(f"⚠️ GPS unstable: freq={freq:.2f}Hz, dist={dist:.2f}m")
            else:
                self.get_logger().info("⏳ Waiting for GPS history...")

            self.last_gps_time = now
            self.last_gps_lat = lat
            self.last_gps_lon = lon

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse GPS NavSatFix: {e}")

    def odom_callback(self, msg: Odometry):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def latlon_to_local(self, lat_ref, lon_ref, lat, lon):
        """Convert lat/lon difference to ENU (meters)."""
        R = 6371000.0
        dlat = math.radians(lat - lat_ref)
        dlon = math.radians(lon - lon_ref)
        x = dlon * math.cos(math.radians(lat_ref)) * R  # East
        y = dlat * R                                    # North
        return x, y

    def timer_callback(self):
        if not self.goal_queue or self.robot_lat is None:
            return

        # If GPS unstable, send fixed forward command
        if not self.gps_stable:
            wp_msg = PointStamped()
            wp_msg.header.stamp = self.get_clock().now().to_msg()
            wp_msg.header.frame_id = "base_link"
            wp_msg.point.x = 4.0
            wp_msg.point.y = 0.0
            wp_msg.point.z = 0.0
            self.waypoint_pub.publish(wp_msg)
            return

        # Distance to final goal
        final_goal_lat, final_goal_lon = self.goal_queue[-1]
        final_dist = haversine_distance(self.robot_lat, self.robot_lon, final_goal_lat, final_goal_lon)
        self.get_logger().info(f"distance to final goal = {final_dist:.2f}m")

        # Current goal
        goal_lat, goal_lon = self.goal_queue[0]
        dist = haversine_distance(self.robot_lat, self.robot_lon, goal_lat, goal_lon)
        self.get_logger().info(f"distance to current goal = {dist:.2f}m")

        msg = Bool()

        # if dist < DIST_THRESHOLD:
        #     self.get_logger().info(f"🎯 Reached waypoint ({goal_lat:.6f}, {goal_lon:.6f}), distance={dist:.2f}m")
            
        #     self.goal_queue.popleft()
        #     if not self.goal_queue:
        #         self.get_logger().info("✅ Path completed, no more waypoints.")
        #         msg.data = True
        #         self.goal_reached_pub.publish(msg)
        #         return
        #     else:
        #         goal_lat, goal_lon = self.goal_queue[0]
        #         self.get_logger().info(f"➡️ Next waypoint: ({goal_lat:.6f}, {goal_lon:.6f})")

        if dist < DIST_THRESHOLD: # or dist > MAX_DIST_THRESHOLD:
            self.get_logger().info(f"🎯 Reached waypoint ({goal_lat:.6f}, {goal_lon:.6f}), distance={dist:.2f}m")
            while(True):
                self.goal_queue.popleft()
                if not self.goal_queue:
                    self.get_logger().info("✅ Path completed, no more waypoints.")
                    msg.data = True
                    self.goal_reached_pub.publish(msg)
                    return
                else:
                    goal_lat, goal_lon = self.goal_queue[0]
                    self.get_logger().info(f"➡️ Next waypoint: ({goal_lat:.6f}, {goal_lon:.6f})")
                    next_dist = haversine_distance(self.robot_lat, self.robot_lon, goal_lat, goal_lon)
                    if( (next_dist > DIST_THRESHOLD) ):
                        break
        msg.data = False
        self.goal_reached_pub.publish(msg)

        # Convert goal to robot frame
        dx, dy = self.latlon_to_local(self.robot_lat, self.robot_lon, goal_lat, goal_lon)
        x_robot = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
        y_robot = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)

        # Save to file
        ts = self.get_clock().now().to_msg()
        self.waypoint_file.write(f"{ts.sec}.{ts.nanosec}, {x_robot:.3f}, {y_robot:.3f}, {dist:.3f}\n")
        self.waypoint_file.flush()
        self.get_logger().info(f"raw goal = {x_robot:.2f} , {y_robot:.2f} , dist: {dist:.2f}")

        x_robot = abs( x_robot )

        if abs(y_robot) > abs(x_robot):
            if(y_robot > 0):
                x_robot = 4.0
                y_robot = 2.0
            else:
                x_robot = 4.0
                y_robot = -2.0

        self.get_logger().info(f"processed goal = {x_robot:.2f} , {y_robot:.2f}")

        # Publish
        wp_msg = PointStamped()
        wp_msg.header.stamp = self.get_clock().now().to_msg()
        wp_msg.header.frame_id = "base_link"
        wp_msg.point.x = x_robot
        wp_msg.point.y = y_robot
        wp_msg.point.z = 0.0
        self.waypoint_pub.publish(wp_msg)

    def destroy_node(self):
        if hasattr(self, "waypoint_file") and not self.waypoint_file.closed:
            self.waypoint_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
