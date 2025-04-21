import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Publisher for robot velocity
        self.publisher_ = self.create_publisher(
            Twist, '/diff_drive/cmd_vel', 10)

        # Subscriber for laser scans
        self.subscription = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',
            self.listener_callback,
            10
        )

        self.subscription  # prevent unused variable warning

        self.laser_range = 0.0
        self.timer = self.create_timer(0.1, self.move_robot)

    def listener_callback(self, msg):
        # Store the laser scan distance (front)
        if len(msg.ranges) > 0:
            self.laser_range = msg.ranges[0]

    def move_robot(self):
        twist = Twist()

        # Simple logic to keep a distance ~0.5m from the wall
        target_distance = 0.5
        error = self.laser_range - target_distance

        kp = 1.0  # proportional gain

        # Control speed based on the error
        twist.linear.x = 0.2
        twist.angular.z = -kp * error

        self.publisher_.publish(twist)
        self.get_logger().info(
            f'Moving with linear x={twist.linear.x:.2f}, angular z={twist.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
