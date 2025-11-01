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
        self.declare_parameter("max_access_cost", int(500))

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
            10,
        )
        
        # Handles: Publishers
        # Path publisher
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10,
        )
        
        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        # Copy robot coordinates from first pose
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        
        # Copy goal coordinates from second pose
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y
        
        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        # Copy costmap data
        self.costmap_ = msg.data
        
        # Copy costmap metadata
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width
        
        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        # Calculate distance from origin
        dx = x - self.costmap_origin_x_
        dy = y - self.costmap_origin_y_
        
        # Convert to cell coordinates and floor
        c = int(floor(dx / self.costmap_resolution_))
        r = int(floor(dy / self.costmap_resolution_))

        return c, r

    # Converts cell column and cell row to world coordinates.
    def CRToXY_(self, c, r):
        # Convert cell to world coordinates at cell center
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_

        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        return (c < 0 or c >= self.costmap_cols_ or 
                r < 0 or r >= self.costmap_rows_)

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
     
        # Initializations ---------------------------------

        # Initialize nodes - create a node for every cell in the costmap
        nodes = []
        for r in range(self.costmap_rows_):
            for c in range(self.costmap_cols_):
                nodes.append(DijkstraNode(c, r))

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]
        
        # Set start node g-cost to 0
        start_node.g = 0.0
        start_node.h = heuristic(rbt_c, rbt_r, goal_c, goal_r)
        start_node.f = start_node.g + start_node.h

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if already expanded
            if node.expanded:
                continue
            
            # Mark node as expanded
            node.expanded = True

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                # Obtain the path from the nodes by following parent pointers
                current = node
                while current is not None:
                    pose = PoseStamped()
                    x, y = self.CRToXY_(current.c, current.r)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    msg_path.poses.append(pose)
                    current = current.parent
                
                # Reverse path so goal is at the end
                msg_path.poses.reverse()

                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = node.c + dc
                nb_r = node.r + dr

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Get neighbor index
                nb_idx = self.CRToIndex_(nb_c, nb_r)

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                # Calculate distance to neighbor
                distance = hypot(dc, dr)
                
                # Calculate tentative g-cost
                g_tentative = node.g + distance * (self.costmap_[nb_idx] + 1)
                h_tentative = heuristic(nb_c, nb_r, goal_c, goal_r)
                f_tentative = g_tentative + h_tentative
                
                # Update if this path is better
                if f_tentative < nb_node.f:
                    nb_node.g = g_tentative
                    nb_node.h = h_tentative
                    nb_node.f = f_tentative
                    nb_node.parent = node
                    heappush(open_list, nb_node)


        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

