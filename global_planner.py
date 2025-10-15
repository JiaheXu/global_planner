import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from example_interfaces.srv import SetInt32
import math
import heapq

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')

        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, 'next_waypoint', 10)

        # Subscriber
        self.create_subscription(NavSatFix, '/gps_raw', self.gps_callback, 10)

        # Service for goal
        self.goal_srv = self.create_service(SetInt32, 'set_goal', self.set_goal_callback)

        # Load map
        self.nodes = self.load_nodes('/mnt/data/nodes.txt')
        self.origin = list(self.nodes.values())[0]
        self.current_pose = None

        # Build adjacency graph using 3m rule
        self.graph = self.build_graph(self.nodes, threshold=3.0)

        self.get_logger().info("✅ MapNavigator started. Call service /set_goal with goal index.")

    def load_nodes(self, path):
        nodes = {}
        with open(path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                idx = int(parts[0])
                lat, lon = float(parts[1]), float(parts[2])
                nodes[idx] = (lat, lon)
        return nodes

    def to_local(self, lat, lon):
        # Flat Earth ENU approx
        lat0, lon0 = self.origin
        R = 6378137.0
        dx = (lon - lon0) * math.pi/180.0 * R * math.cos(lat0*math.pi/180.0)
        dy = (lat - lat0) * math.pi/180.0 * R
        return (dx, dy)

    def gps_callback(self, msg: NavSatFix):
        self.current_pose = self.to_local(msg.latitude, msg.longitude)

    def build_graph(self, nodes, threshold=3.0):
        """Connect nodes if distance <= threshold meters"""
        graph = {i: [] for i in nodes}
        # precompute ENU positions
        enu = {i: self.to_local(lat, lon) for i, (lat, lon) in nodes.items()}
        for i in nodes:
            for j in nodes:
                if i >= j:
                    continue
                d = math.hypot(enu[i][0] - enu[j][0], enu[i][1] - enu[j][1])
                if d <= threshold:
                    graph[i].append(j)
                    graph[j].append(i)
        return graph

    def find_nearest_node(self, pose):
        """Find closest node index to current pose"""
        if pose is None:
            return 0  # fallback
        min_d = float('inf')
        nearest = None
        for idx, (lat, lon) in self.nodes.items():
            x, y = self.to_local(lat, lon)
            d = math.hypot(x - pose[0], y - pose[1])
            if d < min_d:
                min_d = d
                nearest = idx
        return nearest if nearest is not None else 0

    def plan_path(self, start_idx, goal_idx):
        def dist(a, b):
            la, lo = self.nodes[a]
            lb, lo2 = self.nodes[b]
            return math.hypot(la - lb, lo - lo2)

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
            for nb in self.graph[node]:
                if nb not in visited:
                    heapq.heappush(pq, (cost + dist(node, nb), nb, path))
        return []

    def publish_path(self, path_ids):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for pid in path_ids:
            lat, lon = self.nodes[pid]
            x, y = self.to_local(lat, lon)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        if path_ids:
            lat, lon = self.nodes[path_ids[0]]
            x, y = self.to_local(lat, lon)
            wp = PointStamped()
            wp.header.frame_id = "map"
            wp.point.x = x
            wp.point.y = y
            self.waypoint_pub.publish(wp)

    def set_goal_callback(self, request, response):
        goal_idx = request.data
        if goal_idx not in self.nodes:
            response.success = False
            response.message = f"❌ Goal {goal_idx} not found."
            return response

        # pick start as nearest node to robot
        start_idx = self.find_nearest_node(self.current_pose)
        self.get_logger().info(f"Planning path from {start_idx} → {goal_idx}")

        path = self.plan_path(start_idx, goal_idx)
        if not path:
            response.success = False
            response.message = f"⚠️ No path found to {goal_idx}."
            return response

        self.publish_path(path)
        response.success = True
        response.message = f"✅ Path published from {start_idx} to {goal_idx}."
        return response


def main():
    rclpy.init()
    node = MapNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

