#!/usr/bin/env python3
"""
Wall‑following node for WWU CSCI 497F / 597F Lab 2
Keeps ~0.6 m to the LEFT wall with a simple 3‑state controller.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        # ---------- tunable parameters ---------------------------------------
        self.declare_parameter("d_des",        0.60)   # desired distance [m]
        self.declare_parameter("band",         0.05)   # dead‑band  ±
        self.declare_parameter("front_thresh", 0.45)   # obstacle ahead
        self.declare_parameter("v_forward",    0.25)   # m/s
        self.declare_parameter("w_turn",       0.80)   # rad/s
        self.declare_parameter("avg_window",   3)      # beams averaged
        # ---------------------------------------------------------------------

        self.d_des = self.get_parameter("d_des").value
        self.band = self.get_parameter("band").value
        self.front_thresh = self.get_parameter("front_thresh").value
        self.v_fwd = self.get_parameter("v_forward").value
        self.w_turn = self.get_parameter("w_turn").value
        self.N = int(self.get_parameter("avg_window").value)

        self.cmd_pub = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)
        self.create_subscription(LaserScan,
                                 "/diff_drive/scan",
                                 self.scan_cb,
                                 10)

        self.front = float("inf")
        self.left = float("inf")
        self.state = "Forward"          # Forward | TurnLeft | TurnRight

        self.create_timer(0.1, self.control_cb)        # 10 Hz
        self.get_logger().info("✔︎ WallFollower ready")

    # ---------- callbacks ----------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:          # sensor not ready yet
            return

        n = len(msg.ranges)
        # 0 ° is directly ahead,  –90 ° is robot‑left
        idx_front = int((0.0 - msg.angle_min) / msg.angle_increment)
        idx_left = int((-math.pi / 2.0 - msg.angle_min) / msg.angle_increment)

        def mean(idx: int) -> float:
            lo = max(0, idx - self.N // 2)
            hi = min(n - 1, idx + self.N // 2)
            vals = [msg.ranges[k] for k in range(lo, hi + 1)
                    if not math.isinf(msg.ranges[k])]
            return sum(vals) / len(vals) if vals else float("inf")

        self.front = mean(idx_front)
        self.left = mean(idx_left)

    # ------------------------------------------------------------------------
    def control_cb(self):
        # ---- state transitions ---------------------------------------------
        if self.state == "Forward":
            if self.front < self.front_thresh or self.left < self.d_des - self.band:
                self.state = "TurnRight"
            elif self.left > self.d_des + self.band:
                self.state = "TurnLeft"

        elif self.state == "TurnLeft":
            if self.left <= self.d_des + self.band:
                self.state = "Forward"

        elif self.state == "TurnRight":
            if (self.front > self.front_thresh and
                    self.left >= self.d_des - self.band):
                self.state = "Forward"

        # ---- state actions --------------------------------------------------
        cmd = Twist()
        if self.state == "Forward":
            cmd.linear.x = self.v_fwd
            cmd.angular.z = 0.0
        elif self.state == "TurnLeft":
            cmd.linear.x = 0.4 * self.v_fwd
            cmd.angular.z = +self.w_turn
        else:                              # TurnRight
            cmd.linear.x = 0.4 * self.v_fwd
            cmd.angular.z = -self.w_turn

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"{self.state:9s} front={self.front:4.2f} m  "
            f"left={self.left:4.2f} m  → v={cmd.linear.x:4.2f}  w={cmd.angular.z:4.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
