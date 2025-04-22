# wall_follower_node.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

DEG2RAD = math.pi / 180.0


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")

        # ---- Tunable parameters ------------------------------------------------
        # desired left distance [m]
        self.declare_parameter("d_des",          0.50)
        # ± dead‑band around d_des
        self.declare_parameter("band",           0.05)
        # obstacle ahead threshold
        self.declare_parameter("front_thresh",   0.35)
        self.declare_parameter("v_forward",      0.20)   # m/s while driving
        self.declare_parameter("w_turn",         0.60)   # rad/s while turning
        # how many beams to average
        self.declare_parameter("avg_window",         5)
        # ------------------------------------------------------------------------

        self.d_des = self.get_parameter("d_des").value
        self.band = self.get_parameter("band").value
        self.front_thresh = self.get_parameter("front_thresh").value
        self.v_fwd = self.get_parameter("v_forward").value
        self.w_turn = self.get_parameter("w_turn").value
        self.N = int(self.get_parameter("avg_window").value)

        # publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)
        self.sub = self.create_subscription(
            LaserScan, "/diff_drive/scan", self.scan_cb, 10
        )

        # internal state
        self.state = "Forward"
        self.front_dist = float("inf")
        self.left_dist = float("inf")

        # 10 Hz control loop
        self.create_timer(0.1, self.control_cb)

        self.get_logger().info("WallFollower node started")

    # ------------------------------------------------------------------------- #
    #  Callbacks
    # ------------------------------------------------------------------------- #
    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # Compute indices for 0 deg (front) and +90 deg (left)
        front_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        left_idx = int((math.pi/2 - msg.angle_min) / msg.angle_increment)

        def mean_window(center):
            lo = max(0, center - self.N // 2)
            hi = min(n - 1, center + self.N // 2)
            vals = [msg.ranges[i]
                    for i in range(lo, hi + 1) if not math.isinf(msg.ranges[i])]
            return sum(vals) / len(vals) if vals else float("inf")

        self.front_dist = mean_window(front_idx)
        self.left_dist = mean_window(left_idx)


def control_cb(self):
    """Finite‑state controller."""
    # -------- State transitions ----------
    if self.state == "Forward":
        if self.front_dist < self.front_thresh:
            self.state = "TurnRight"  # Wall ahead, turn away
        elif self.left_dist < self.d_des - self.band:
            self.state = "TurnRight"  # Too close to wall, turn away
        elif self.left_dist > self.d_des + self.band:
            self.state = "TurnLeft"   # Too far from wall, turn toward

    elif self.state == "TurnLeft":
        # Only return to Forward if we're at the right distance
        if self.left_dist <= self.d_des + self.band:
            self.state = "Forward"

    elif self.state == "TurnRight":
        # For a front obstacle, make sure we've cleared it before returning to Forward
        if self.front_dist > self.front_thresh and self.left_dist >= self.d_des - self.band:
            self.state = "Forward"

    # -------- State actions ---------------
    cmd = Twist()
    if self.state == "Forward":
        cmd.linear.x = self.v_fwd
        cmd.angular.z = 0.0
    elif self.state == "TurnLeft":
        cmd.linear.x = self.v_fwd * 0.5  # Keep moving forward while turning
        cmd.angular.z = +self.w_turn
    elif self.state == "TurnRight":
        cmd.linear.x = self.v_fwd * 0.5  # Keep moving forward while turning
        cmd.angular.z = -self.w_turn

    self.cmd_pub.publish(cmd)
    self.get_logger().info(
        f"{self.state:<10}  front={self.front_dist:4.2f}  left={self.left_dist:4.2f}  "
        f"→ v={cmd.linear.x:4.2f}  w={cmd.angular.z:4.2f}"
    )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
