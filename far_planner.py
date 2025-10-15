import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from math import radians, cos, sin, asin, sqrt

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        # Load waypoints from file
        self.waypoints = self.load_waypoints("nodes.txt")
        self.current_idx = 0

        # Parameters
        self.threshold_m = 1.0  # distance threshold in meters

        # ROS interfaces
        self.subscription = self.create_subscription(
            NavSatFix, '/gps_raw', self.gps_callback, 10)
        self.publisher = self.create_publisher(
            PointStamped, '/way_point', 10)

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, "r") as f:
            for line in f:
                idx, lat, lon = line.strip().split(",")
                waypoints.append((float(lat), float(lon)))
        return waypoints

    def haversine(self, lat1, lon1, lat2, lon2):
        """Return distance in meters between two GPS coordinates."""
        R = 6371000.0  # Earth radius
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        return R * c

    def gps_callback(self, msg: NavSatFix):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        lat, lon = msg.latitude, msg.longitude
        target_lat, target_lon = self.waypoints[self.current_idx]

        dist = self.haversine(lat, lon, target_lat, target_lon)

        if dist < self.threshold_m:
            self.get_logger().info(
                f"Reached waypoint {self.current_idx}, moving to next."
            )
            self.current_idx += 1

            if self.current_idx >= len(self.waypoints):
                self.get_logger().info("Final waypoint reached.")
                return

        # Always publish the next waypoint (if any left)
        if self.current_idx < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.current_idx]
            wp_msg = PointStamped()
            wp_msg.header.stamp = self.get_clock().now().to_msg()
            wp_msg.header.frame_id = "map"
            wp_msg.point.x = target_lat
            wp_msg.point.y = target_lon
            wp_msg.point.z = 0.0
            self.publisher.publish(wp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

