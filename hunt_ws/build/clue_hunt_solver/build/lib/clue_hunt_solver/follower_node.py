#!/usr/bin/env python3
"""
Clue Chain Hunt - FOLLOWER node (camera + wheel odometry only).

Idea: see the leader's ArUco tag (id 49) -> leader position in `follower/odom` ->
drop "breadcrumbs" along the leader's path -> drive along the breadcrumbs (pure pursuit)
and keep the gap between 0.6 and 2.0 m.

Changes vs the old version (so it can keep up):
  * speed = leader speed (feed-forward) + P-term on the gap
  * keeps driving along the breadcrumbs for a few seconds when the tag is lost
  * pure pursuit on the trail (nearest crumb + lookahead) instead of "oldest crumb"
  * smooth slow-down on turns (cos of heading error) instead of stop / x0.3
  * image callback and control loop run in different threads
Only /follower/... topics and TF frames starting with `follower/` are used.
"""
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .common import ArucoDetector, image_to_bgr, marker_pose, quat_to_R, wrap

ODOM = 'follower/odom'
BASE = 'follower/base_footprint'


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.declare_parameter('tag_id', 49)
        self.declare_parameter('tag_size', 0.12)
        self.declare_parameter('tag_behind_centre', 0.21)
        self.declare_parameter('d_stop', 0.9)       # closer than this: stand still
        self.declare_parameter('d_des', 1.2)        # gap we want while moving
        self.declare_parameter('kp_gap', 1.5)       # extra speed per metre of gap error
        self.declare_parameter('lookahead', 0.7)    # pure pursuit distance along the trail
        self.declare_parameter('lost_drive_time', 3.0)  # keep following crumbs this long
        self.declare_parameter('v_max', 0.7)
        self.declare_parameter('w_max', 1.5)
        self.tag_id = self.get_parameter('tag_id').value
        self.tag_size = self.get_parameter('tag_size').value
        self.behind = self.get_parameter('tag_behind_centre').value
        self.d_stop = self.get_parameter('d_stop').value
        self.d_des = self.get_parameter('d_des').value
        self.kp_gap = self.get_parameter('kp_gap').value
        self.lookahead = self.get_parameter('lookahead').value
        self.lost_drive_time = self.get_parameter('lost_drive_time').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.detector = ArucoDetector()
        self.K, self.D = None, np.zeros(5)
        self.leader = None                # last known leader centre in follower/odom
        self.last_seen = 0.0
        self.leader_speed = 0.0           # smoothed estimate of leader speed (m/s)
        self._prev = None                 # (position, time) of previous detection
        self.trail = deque(maxlen=300)    # breadcrumbs (x, y) in follower/odom
        self.lock = threading.Lock()
        self.last_status = ''

        cam_group = MutuallyExclusiveCallbackGroup()
        ctl_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, '/follower/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data, callback_group=cam_group)
        self.create_subscription(CameraInfo, '/follower/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data, callback_group=cam_group)
        self.create_subscription(String, '/leader/status', self.on_status, 10,
                                 callback_group=cam_group)
        self.cmd_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)
        self.create_timer(0.05, self.control, callback_group=ctl_group)
        self.get_logger().info('follower_node started')

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if len(msg.d) >= 4:
                self.D = np.array(msg.d, dtype=np.float64)

    def on_status(self, msg):
        self.last_status = msg.data

    def lookup(self, target, source, stamp=None):
        tf = None
        try:
            if stamp is not None:
                try:
                    tf = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp))
                except TransformException:
                    tf = None
            if tf is None:
                tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def on_image(self, msg):
        if self.K is None:
            return
        img = image_to_bgr(msg)
        corners, ids = self.detector.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        for c4, mid in zip(corners, ids):
            if mid != self.tag_id:
                continue
            pose = marker_pose(c4, self.tag_size, self.K, self.D)
            if pose is None:
                continue
            rvec, tv = pose
            if tv[2] <= 0 or np.linalg.norm(tv) > 6.0:
                continue
            Rcm, _ = cv2.Rodrigues(rvec)
            n = Rcm[:, 2]                                   # tag normal (points out of its face)
            if np.dot(n, tv) > 0:                           # must face the camera
                n = -n
            centre_cam = tv - self.behind * n               # leader centre is 0.21 m ahead of its tag
            T = self.lookup(ODOM, 'follower/cam_optical_link', msg.header.stamp)
            if T is None:
                return
            R, t = T
            p = (R @ centre_cam + t)[:2].copy()
            now = time.monotonic()

            # estimate how fast the leader is moving (low-pass filtered)
            if self._prev is not None:
                dt = now - self._prev[1]
                if 0.03 < dt < 0.5:
                    inst = np.linalg.norm(p - self._prev[0]) / dt
                    inst = min(inst, 1.5)                   # ignore detection glitches
                    self.leader_speed = 0.7 * self.leader_speed + 0.3 * inst
            self._prev = (p, now)

            with self.lock:
                self.leader = p
                self.last_seen = now
                if not self.trail or np.linalg.norm(self.trail[-1] - p) > 0.12:
                    self.trail.append(p.copy())
            return

    def pick_target(self, x, y):
        """Pure pursuit: drop crumbs we already passed, aim a bit ahead on the trail."""
        with self.lock:
            pts = list(self.trail)
            leader = None if self.leader is None else self.leader.copy()
            if not pts:
                return leader
            # nearest crumb among the oldest few (avoids jumping if the path crosses itself)
            search = pts[:40]
            dists = [math.hypot(c[0] - x, c[1] - y) for c in search]
            i = int(np.argmin(dists))
            for _ in range(i):
                self.trail.popleft()
            pts = pts[i:]
        # walk along the trail until we are `lookahead` away from the robot
        for c in pts:
            if math.hypot(c[0] - x, c[1] - y) >= self.lookahead:
                return c
        return pts[-1]

    def control(self):
        T = self.lookup(ODOM, BASE)
        cmd = Twist()
        if T is None:
            self.cmd_pub.publish(cmd)
            return
        R, t = T
        x, y, yaw = t[0], t[1], math.atan2(R[1, 0], R[0, 0])
        now = time.monotonic()

        if self.leader is None:                             # never seen it: look around slowly
            cmd.angular.z = 0.4
            self.cmd_pub.publish(cmd)
            return

        leader = self.leader
        lead_ang = wrap(math.atan2(leader[1] - y, leader[0] - x) - yaw)
        d = math.hypot(leader[0] - x, leader[1] - y)
        since_seen = now - self.last_seen
        lost = since_seen > 0.6

        if lost and since_seen > self.lost_drive_time:      # really lost: spin to search
            cmd.angular.z = float(np.clip(1.5 * lead_ang, -0.8, 0.8)) if abs(lead_ang) > 0.1 else 0.5
            self.cmd_pub.publish(cmd)
            return

        if not lost and d <= self.d_stop:                   # close enough: stand still, face leader
            cmd.angular.z = float(np.clip(1.5 * lead_ang, -self.w_max, self.w_max))
            self.cmd_pub.publish(cmd)
            return

        target = self.pick_target(x, y)
        err = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)

        if lost:
            # tag out of view: keep following the crumbs towards where the leader went
            v = 0.3 if d > self.d_stop else 0.0
        else:
            # match the leader's speed, then correct the gap
            v = self.leader_speed + self.kp_gap * (d - self.d_des)
            v = float(np.clip(v, 0.0, self.v_max))
        v *= max(0.0, math.cos(err)) if abs(err) < 1.2 else 0.0   # smooth slow-down on turns

        cmd.linear.x = v
        cmd.angular.z = float(np.clip(2.5 * err, -self.w_max, self.w_max))
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = FollowerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
