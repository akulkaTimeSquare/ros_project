import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np


class DataProc(Node):
    def __init__(self):
        super().__init__('data_proc')
        self.safe_dist = 1.0
        self.max_speed = 0.5

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.left = None
        self.right = None
        self.front = None

        self.bridge = CvBridge()
        self.edge_ratio_thresh = 0.001
        self.low_obstacle_fl = None
        self.low_obstacle_right = None

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # front + left
        frame_fl = frame[int(h * 0.3):h, int(w * 0.3):w]
        gray_fl = cv2.cvtColor(frame_fl, cv2.COLOR_BGR2GRAY)
        edges_fl = cv2.Canny(gray_fl, 35, 200)
        ratio_fl = np.count_nonzero(edges_fl) / edges_fl.size
        self.low_obstacle_fl = ratio_fl > self.edge_ratio_thresh

        # right
        frame_right = frame[int(h * 0.3):h, 0:int(w * 0.3)]
        gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
        edges_right = cv2.Canny(gray_right, 35, 200)
        ratio_right = np.count_nonzero(edges_right) / edges_right.size
        self.low_obstacle_right = ratio_right > self.edge_ratio_thresh

    def lidar_callback(self, msg):
        scan = msg
        ranges = scan.ranges
        n = len(ranges)
        self.left = self.sector_min(ranges, 0, n // 3)
        self.right = self.sector_min(ranges, 2 * n // 3, n)
        self.front = self.sector_min(ranges, n // 3, 2 * n // 3)

    def sector_min(self, ranges, a, b):
        sector = np.array(ranges[a:b])
        sector = np.where(
            (sector < 0.08) | np.isinf(sector), 10.0, sector
        )
        return np.min(sector) if len(sector) else 10.0

    def control_loop(self):
        if self.low_obstacle_fl is None or self.low_obstacle_right is None:
            return
        if self.left is None or self.right is None or self.front is None:
            return
        
        linear = 0.0
        angular = 0.0

        # camera control
        if self.low_obstacle_fl:
            linear = 0.0
            angular = 1.5

        # lidar control
        elif self.front < self.safe_dist or self.left < 0.5:
            linear = 0.05
            angular = 2.0

        elif self.right < 0.5:
            linear = 0.05
            angular = -2.0

        else:
            linear = self.max_speed
            angular = (self.right - self.left) * 1.5

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DataProc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
