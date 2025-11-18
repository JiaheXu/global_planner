#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import math

class FarPlanner(Node):
    def __init__(self):
        super().__init__('far_planner')

        # Internal states
        self.goal_lat = None
        self.goal_lon = None
        self.robot_lat = None
        self.robot_lon = None
        self.robot_yaw = 0.0  # from odometry

        # Subscribers
        self.create_subscription(NavSatFix, 'nav_goal', self.goal_callback, 10)
        self.create_subscription(NavSatFix, 'gps_raw', self.gps_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publisher
        self.waypoint_pub = self.create_publisher(PointStamped, 'waypoint', 10)

        # Timer for 5Hz output
        self.create_timer(0.2, self.timer_callback)

        self.get_logger().info("✅ FarPlanner node started (listening to nav_goal, gps_raw, odom).")

    def goal_callback(self, msg: NavSatFix):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude
        self.get_logger().info(f"Received nav_goal: lat={self.goal_lat}, lon={self.goal_lon}")

    def gps_callback(self, msg: NavSatFix):
        self.robot_lat = msg.latitude
        self.robot_lon = msg.longitude

    def odom_callback(self, msg: Odometry):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def latlon_to_local(self, lat_ref, lon_ref, lat, lon):
        """Convert lat/lon difference to ENU (meters)."""
        R = 6378137.0  # Earth radius
        dlat = math.radians(lat - lat_ref)
        dlon = math.radians(lon - lon_ref)
        x = dlon * math.cos(math.radians(lat_ref)) * R  # East
        y = dlat * R                                    # North
        return x, y

    def timer_callback(self):
        if self.goal_lat is None or self.robot_lat is None:
            return  # No data yet

        # Convert goal to ENU relative to robot GPS
        dx, dy = self.latlon_to_local(self.robot_lat, self.robot_lon,
                                      self.goal_lat, self.goal_lon)

        # Rotate into robot frame using odometry yaw
        x_robot = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
        y_robot = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)

        # Publish as PointStamped
        wp_msg = PointStamped()
        wp_msg.header.stamp = self.get_clock().now().to_msg()
        wp_msg.header.frame_id = "base_link"  # robot frame
        wp_msg.point.x = x_robot
        wp_msg.point.y = y_robot
        wp_msg.point.z = 0.0

        self.waypoint_pub.publish(wp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
