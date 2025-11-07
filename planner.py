from heapq import heappush, heappop
from math import hypot, floor, inf

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
  
def heuristic(c, r, goal_c, goal_r):
    return hypot(c - goal_c, r - goal_r)

class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):
        return self.f < other.f

class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # Path request subscriber
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10
        )
        
        # Handles: Publishers
        # Path publisher
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10
        )
        
        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    def risk_metric(self, c, r):
        max_cost = 100
        neighbor_offsets = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, 1), (-1, -1), (1, -1)
        ]
        risk = 0.0
        for dc, dr in neighbor_offsets:
            nc = c + dc
            nr = r + dr
            if self.outOfMap_(nc, nr):
                continue
            idx = self.CRToIndex_(nc, nr)
            neighbor_cost = self.costmap_[idx]
            risk += neighbor_cost / max_cost
        return min(risk / len(neighbor_offsets), 1.0)

    # Callback to copy robot and goal coordinates from requested path message
    def callbackSubPathRequest_(self, msg: Path):   
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y
        
        self.has_new_request_ = True

    # Callback to copy global costmap data and metadata
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        self.costmap_ = msg.data
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width
        
        self.received_map_ = True

    # Timer callback to trigger path planning on new request
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return

        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        self.has_new_request_ = False

    # Convert world coordinates to costmap cell indices
    def XYToCR_(self, x, y):
        dx = x - self.costmap_origin_x_
        dy = y - self.costmap_origin_y_
        c = int(floor(dx / self.costmap_resolution_))
        r = int(floor(dy / self.costmap_resolution_))
        return c, r

    # Convert cell indices to world coordinates at cell center
    def CRToXY_(self, c, r):
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_
        return x, y

    # Converts cell indices to flattened costmap array index
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Checks if given cell indices are outside costmap bounds
    def outOfMap_(self, c, r):
        return (c < 0 or c >= self.costmap_cols_ or 
                r < 0 or r >= self.costmap_rows_)

    # Main Dijkstra / A* path planning method with hybrid cost function
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        w1 = 0.00001# Weight for costmap cell cost
        w2 = 0.00001  # Weight for risk metric

        nodes = []
        for r in range(self.costmap_rows_):
            for c in range(self.costmap_cols_):
                nodes.append(DijkstraNode(c, r))

        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]
        
        start_node.g = 0.0
        start_node.h = heuristic(rbt_c, rbt_r, goal_c, goal_r)
        start_node.f = start_node.g + start_node.h

        open_list = []
        heappush(open_list, start_node)

        while len(open_list) > 0:
            node = heappop(open_list)

            if node.expanded:
                continue
            node.expanded = True

            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                current = node
                while current is not None:
                    pose = PoseStamped()
                    x, y = self.CRToXY_(current.c, current.r)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    msg_path.poses.append(pose)
                    current = current.parent
                
                msg_path.poses.reverse()
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )
                return

            for dc, dr in [
                (1, 0), (1, 1), (0, 1), (-1, 1),
                (-1, 0), (-1, -1), (0, -1), (1, -1),
            ]:
                nb_c = node.c + dc
                nb_r = node.r + dr

                if self.outOfMap_(nb_c, nb_r):
                    continue

                nb_idx = self.CRToIndex_(nb_c, nb_r)
                nb_node = nodes[nb_idx]

                if nb_node.expanded:
                    continue

                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                distance = hypot(dc, dr)
                cell_risk = self.risk_metric(nb_c, nb_r)

                g_tentative = node.g + distance * (w1 * self.costmap_[nb_idx] + w2 * cell_risk + 1)
                if g_tentative < nb_node.g:
                    nb_node.g = g_tentative
                    nb_node.h = heuristic(nb_c, nb_r, goal_c, goal_r)
                    nb_node.f = nb_node.g + nb_node.h
                    nb_node.parent = node
                    heappush(open_list, nb_node)

        self.get_logger().warn("No Path Found!")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

