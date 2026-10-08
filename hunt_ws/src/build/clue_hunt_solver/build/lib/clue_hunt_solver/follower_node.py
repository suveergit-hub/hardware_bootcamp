#!/usr/bin/env python3
"""
Clue Chain Hunt - FOLLOWER node. Your solution goes here.

Run (with the simulation running):
  ros2 run clue_hunt_solver follower_node

The follower is a second robot with ONLY:
  /follower/camera/image_raw, /follower/camera/camera_info   RGB camera (tilted 8 deg UP,
                                                              frame follower/cam_optical_link)
  /follower/odom                                             wheel odometry
  TF frames that start with "follower/"                      follower/odom -> follower/base_footprint -> ...
  /leader/status (optional)                                  MOVING / SEARCHING / READING / DONE
and drives with:
  /follower/cmd_vel                                          geometry_msgs/Twist

It has NO LiDAR and NO map. It must NOT use the leader's /odom, /tf frames, /cmd_vel, Nav2 topics
or any Gazebo ground truth.

The leader carries an ArUco marker on its back: DICT_4X4_50, id 49, side 0.12 m,
centre 0.30 m above the floor and 0.21 m behind the leader's centre, facing backwards.

Goal: stay 0.6 - 2.0 m from the leader during the whole hunt, never touch a wall or the leader,
and finish within 1.5 m of the treasure.
Hint: aiming straight at the leader cuts corners into walls...
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.create_subscription(Image, '/follower/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/follower/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/leader/status', self.on_status, 10)
        self.cmd_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)
        self.create_timer(0.05, self.control)
        self.get_logger().info('follower_node started - find the leader!')

    def on_image(self, msg):
        # TODO 1: detect the leader's tag (id 49) and estimate its pose (solvePnP, 0.12 m)
        # TODO 2: express the leader's position in follower/odom (tf2)
        pass

    def on_info(self, msg):
        pass

    def on_status(self, msg):
        pass

    def control(self):
        # TODO 3: follow the leader safely and publish /follower/cmd_vel
        self.cmd_pub.pu
        pass


def main():
    rclpy.init()
    node = FollowerNode()
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
