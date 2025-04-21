import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        # Declare and read ROS parameters
        self.declare_parameter('d_des', 0.5)
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('front_thresh', 0.5)
        self.declare_parameter('L_low', 0.2)
        self.declare_parameter('L_high', 1.0)
        self.declare_parameter('turn_speed', 0.5)

        self.d_des = self.get_parameter('d_des').value
        self.kp = self.get_parameter('Kp').value
        self.front_thresh = self.get_parameter('front_thresh').value
        self.L_low = self.get_parameter('L_low').value
        self.L_high = self.get_parameter('L_high').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Publisher and subscriber
        self.publisher_ = self.create_publisher(
            Twist, '/diff_drive/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize state and sensor readings
        self.front = float('inf')
        self.left = float('inf')
        self.state = 'Forward'

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.move_robot)

    def listener_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        front_idx = n // 2
        left_idx = 3 * n // 4
        raw_front = msg.ranges[front_idx]
        raw_left = msg.ranges[left_idx]
        # Store raw measurements (could filter here)
        self.front = raw_front
        self.left = raw_left

    def move_robot(self):
        twist = Twist()

        # --- State transition ---
        if self.state == 'Forward':
            if self.front < self.front_thresh or self.left <= self.L_low:
                self.state = 'RotateRight'
            elif self.left >= self.L_high:
                self.state = 'RotateLeft'
        else:
            # After one rotation tick, return to forward
            self.state = 'Forward'

        # --- State action ---
        if self.state == 'Forward':
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif self.state == 'RotateRight':
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        elif self.state == 'RotateLeft':
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed

        self.publisher_.publish(twist)
        self.get_logger().info(
            f"State={self.state}, front={self.front:.2f}, left={self.left:.2f} -> "
            f"cmd(linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
