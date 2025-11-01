from math import hypot, atan2, inf, cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value

        # Handles: Topic Subscribers
        # Path subscriber
        self.sub_path_ = self.create_subscription(
            Path,
            "path",
            self.callbackSubPath_,
            10,
        )

        # Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            10,
        )

        # Handles: Topic Publishers
        # Command velocities publisher
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped,
            "cmd_vel",
            10,
        )

        # Lookahead point publisher
        self.pub_lookahead_ = self.create_publisher(
            PoseStamped,
            "lookahead",
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False

    # Callbacks =============================================================
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:  # not msg.poses is fine but not clear
            self.get_logger().warn(f"Received path message is empty!")
            return  # do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.

        # Copy the array from the path
        self.path_poses_ = msg.poses
        self.received_path_ = True

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        # Copy robot position
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        delta_y = 2.0 * (q.w * q.z + q.x * q.y)
        delta_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rbt_yaw_ = atan2(delta_y, delta_x)

        self.received_odom_ = True

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Step 1: Find the point along the path that is closest to the robot
        min_distance = float('inf')  # Initialize minimum distance to infinity
        closest_idx = 0  # Initialize index of closest point
        
        # Iterate through all path points to find the closest one
        for i, pose in enumerate(self.path_poses_):
            # Extract point coordinates from path
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y
            
            # Calculate Euclidean distance from robot to this point
            distance = hypot(point_x - self.rbt_x_, point_y - self.rbt_y_)
            
            # Update closest point if this one is nearer
            if distance < min_distance:
                min_distance = distance
                closest_idx = i

        # Step 2: From the closest point, iterate towards the goal and find the first point 
        # that is at least lookahead_distance_ away from the robot
        lookahead_idx = len(self.path_poses_) - 1  # Default to goal point (last point)
        
        # Search from closest point to goal for lookahead point
        for i in range(closest_idx, len(self.path_poses_)):
            # Extract point coordinates
            point_x = self.path_poses_[i].pose.position.x
            point_y = self.path_poses_[i].pose.position.y
            
            # Calculate distance from robot to this point
            distance = hypot(point_x - self.rbt_x_, point_y - self.rbt_y_)
            
            # If point is at least lookahead_distance_ away, use it as lookahead point
            if distance >= self.lookahead_distance_:
                lookahead_idx = i
                break  # Found lookahead point, exit loop

        # Get the lookahead coordinates
        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish the lookahead coordinates for visualization in RViz
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Return the coordinates
        return lookahead_x, lookahead_y

    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        # Step 1: Check if required data has been received
        if not self.received_odom_ or not self.received_path_:
            return  # Return silently if path or odom is not received

        # Step 2: Get lookahead point coordinates in world frame
        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        # Step 3: Calculate distance from robot to lookahead point
        # This is the Euclidean distance L in the equations
        distance_to_lookahead = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)

        # Step 4: Stop the robot if close to the lookahead point
        if distance_to_lookahead < self.stop_thres_:
            # Publish zero velocities to stop the robot
            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = 0.0
            msg_cmd_vel.twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            return  # Exit function after stopping

        # Step 5: Transform lookahead point from world frame to robot frame
        # x' = (xl - xr) * cos(φr) + (yl - yr) * sin(φr)
        # y' = -(xl - xr) * sin(φr) + (yl - yr) * cos(φr)
        dx = lookahead_x - self.rbt_x_  # Difference in x (world frame)
        dy = lookahead_y - self.rbt_y_  # Difference in y (world frame)
        
        # Apply rotation to transform to robot frame
        x_prime = dx * cos(self.rbt_yaw_) + dy * sin(self.rbt_yaw_)
        y_prime = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)

        # Step 6: Calculate curvature using pure pursuit formula
        # c = 2 * y' / L²
        # where y' is lateral distance in robot frame, L is distance to lookahead
        curvature = (2.0 * y_prime) / (distance_to_lookahead ** 2)

        # Step 7: Calculate velocities
        # Linear velocity is constant (lookahead_lin_vel_)
        lin_vel = self.lookahead_lin_vel_
        if x_prime < 0:
            lin_vel *= -1.0
            curvature *= -1.0   # reverse curvature sign to steer correctly
        
        # Angular velocity is linear velocity times curvature: ω = v * c
        ang_vel = lin_vel * curvature

        # Step 8: Saturate (constrain) velocities to maximum allowed values
        # Constrain linear velocity between -max and +max
        lin_vel = max(-self.max_lin_vel_, min(lin_vel, self.max_lin_vel_))
        
        # Constrain angular velocity between -max and +max
        ang_vel = max(-self.max_ang_vel_, min(ang_vel, self.max_ang_vel_))

        # Step 9: Publish the velocities to command the robot
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

