import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarHandler(Node):

    def __init__(self):
        super().__init__('lidar_min_distance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',
            self.lidar_callback,
            10
        )
        self.get_logger().info("initialized. Subscribed /scan")

    def lidar_callback(self, msg: LaserScan):
        # Convert list → numpy array
        ranges = np.array(msg.ranges)

        # Remove invalid values (inf, nan, zeros)
        valid = ranges[np.isfinite(ranges)]
        # ignore tiny noise values
        valid = valid[valid > 0.01]  

        if len(valid) == 0:
            self.get_logger().info("No valid LiDAR points")
            return

        min_dist = float(np.min(valid))

        self.get_logger().info(f"Min distance: {min_dist:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
