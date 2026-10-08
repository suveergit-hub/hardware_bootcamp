#!/usr/bin/env python3
"""
Clue Chain Hunt - LEADER node. Your solution goes here.

Run (with the simulation + Nav2 already running):
  ros2 run clue_hunt_solver hunt_node

Topics the referee listens to (you MUST publish these):
  /hunt/clues     std_msgs/String            full QR text of every VALID clue you read, in chain order,
                                             e.g. "HUNT:2:7F49:PILLAR RED"
  /hunt/boards    std_msgs/String            "<id> <x> <y>" - your estimate of each board's ArUco centre
                                             in the map frame, e.g. "2 1.65 -3.88"
  /hunt/treasure  geometry_msgs/PoseStamped  (frame "map") the treasure position, once, after the
                                             leader has driven onto it
Topic the follower may listen to (optional):
  /leader/status  std_msgs/String            MOVING / SEARCHING / READING / DONE

Leader inputs:
  /camera/image_raw, /camera/camera_info   RGB camera (frame cam_optical_link, tilted 5 deg down)
  /scan                                    360 deg LiDAR (frame lidar_link)
  /odom, /imu, /tf                         odometry, IMU, TF (map -> odom -> base_footprint -> ...)
  Nav2 action /navigate_to_pose            (or nav2_simple_commander.BasicNavigator)

Clue text:  HUNT:<id>:<token>:[TREASURE ]<command>
  token = first 4 hex characters (upper case) of SHA-1 of the PREVIOUS valid clue text
          (for board 1: SHA-1 of "START")  ->  python: hashlib.sha1(prev.encode()).hexdigest()[:4].upper()
  A board whose token does not match is a LOOK-ALIKE: ignore it and keep searching.

Not allowed: Gazebo ground truth (/model/...), hard-coded board / pillar / treasure positions.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String


class HuntNode(Node):
    def __init__(self):
        super().__init__('hunt_node')
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.clue_pub = self.create_publisher(String, '/hunt/clues', 10)
        self.board_pub = self.create_publisher(String, '/hunt/boards', 10)
        self.treasure_pub = self.create_publisher(PoseStamped, '/hunt/treasure', 10)
        self.status_pub = self.create_publisher(String, '/leader/status', 10)
        self.get_logger().info('hunt_node started - time to write some code!')

    def on_image(self, msg):
        # TODO 1: convert to OpenCV (cv_bridge), detect ArUco markers (DICT_4X4_50, 0.24 m).
        #         Several boards can carry the SAME id (look-alikes).
        # TODO 2: decode the QR code next to the marker (cv2.QRCodeDetector) and check the token
        pass

    def on_info(self, msg):
        # TODO: keep the camera matrix K for solvePnP
        pass

    def on_scan(self, msg):
        pass

    # TODO 3: estimate the board pose with solvePnP, transform it into the map frame (tf2),
    #         publish it on /hunt/boards
    # TODO 4: parse the clue (GOTO / PILLAR / BETWEEN / REL, optional TREASURE prefix)
    # TODO 5: navigate with Nav2, search for the next board, repeat (drive smoothly - the
    #         follower has to keep up!)
    # TODO 6: drive onto the treasure and publish /hunt/treasure


def main():
    rclpy.init()
    node = HuntNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
