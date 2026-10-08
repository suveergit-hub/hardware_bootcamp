#!/usr/bin/env python3
"""
Clue Chain Hunt - LEADER node.

Pipeline:  see ArUco -> estimate board pose (solvePnP + TF) -> drive close ->
           decode QR -> check chain token -> parse clue -> pick next search area ->
           Nav2 there -> spin/search -> repeat -> drive onto treasure.

Pillars (needed for PILLAR / BETWEEN clues) are found at run time from the camera
(colour blob) + LiDAR (range), never hard-coded.
"""
import hashlib
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformException, TransformListener

from .common import (R_MB, ArucoDetector, decode_qr_near_marker, image_to_bgr,
                     marker_pose, quat_to_R, wrap, yaw_to_quat)

PILLAR_HSV = {
    'RED': [((0, 130, 50), (8, 255, 255)), ((172, 130, 50), (180, 255, 255))],
    'GREEN': [((50, 130, 40), (72, 255, 255))],
    'BLUE': [((105, 130, 40), (126, 255, 255))],
}


def token_of(prev_text):
    return hashlib.sha1(prev_text.encode()).hexdigest()[:4].upper()


class HuntNode(Node):
    def __init__(self):
        super().__init__('hunt_node')
        self.declare_parameter('tag_size', 0.24)
        self.declare_parameter('standoff', 1.2)
        self.declare_parameter('search_radius', 1.3)
        self.tag_size = self.get_parameter('tag_size').value
        self.standoff = self.get_parameter('standoff').value
        self.ring_r = self.get_parameter('search_radius').value

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.K = None
        self.D = np.zeros(5)
        self.scan = None
        self.map_info = None
        self.blocked = None
        self.candidates = []            # every board seen so far
        self.pillar_obs = {c: [] for c in PILLAR_HSV}
        self.expected_id = 1
        self.lock = threading.Lock()
        self.img_lock = threading.Lock()
        self.detector = ArucoDetector()
        self.qr = cv2.QRCodeDetector()

        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE), callback_group=cb)
        self.clue_pub = self.create_publisher(String, '/hunt/clues', 10)
        self.board_pub = self.create_publisher(String, '/hunt/boards', 10)
        self.treasure_pub = self.create_publisher(PoseStamped, '/hunt/treasure', 10)
        self.status_pub = self.create_publisher(String, '/leader/status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.get_logger().info('hunt_node started')

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if len(msg.d) >= 4:
                self.D = np.array(msg.d, dtype=np.float64)

    def on_scan(self, msg):
        self.scan = msg

    def on_map(self, msg):
        h, w = msg.info.height, msg.info.width
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        occ = ((data > 50) | (data < 0)).astype(np.uint8)       # unknown = blocked
        r = max(1, int(math.ceil(0.35 / msg.info.resolution)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.blocked = cv2.dilate(occ, kernel).astype(bool)
        self.map_info = (msg.info.resolution, msg.info.origin.position.x,
                         msg.info.origin.position.y, w, h)

    def lookup(self, target, source, stamp=None):
        """Return (R, t) of `source` expressed in `target`, or None."""
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

    def robot_pose(self):
        T = self.lookup('map', 'base_link')
        if T is None:
            return None
        R, t = T
        return t[0], t[1], math.atan2(R[1, 0], R[0, 0])

    # ------------------------------------------------------------------ vision
    def on_image(self, msg):
        if self.K is None or not self.img_lock.acquire(blocking=False):
            return
        try:
            img = image_to_bgr(msg)
            T = self.lookup('map', 'cam_optical_link', msg.header.stamp)   # fixed name: Gazebo may publish a scoped frame_id
            if T is None:
                return
            Rmc, tmc = T
            self.detect_pillars(img, Rmc, tmc)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids = self.detector.detect(gray)
            for c4, mid in zip(corners, ids):
                pose = marker_pose(c4, self.tag_size, self.K, self.D)
                if pose is None:
                    continue
                rvec, tv = pose
                dist = float(np.linalg.norm(tv))
                Rcm, _ = cv2.Rodrigues(rvec)
                n_cam = Rcm @ R_MB[:, 0]                   # board +X in camera frame
                if tv[2] <= 0 or dist > 5.5 or float(np.dot(n_cam, -tv)) <= 0:
                    continue                               # behind / too far / flipped PnP solution
                pos = Rmc @ tv + tmc
                nrm = Rmc @ n_cam
                cand = self.update_candidate(mid, pos, nrm, dist)
                if cand is not None and mid == self.expected_id and dist < 2.8:
                    text = decode_qr_near_marker(img, rvec, tv, self.K, self.D, self.qr)
                    if text:
                        with self.lock:
                            cand['qr'] = text.strip()
        finally:
            self.img_lock.release()

    def update_candidate(self, mid, pos, nrm, dist):
        if dist > 3.8:
            return None
        with self.lock:
            for c in self.candidates:
                if c['id'] == mid and np.linalg.norm(c['pos'][:2] - pos[:2]) < 0.9:
                    break
            else:
                c = {'id': mid, 'obs': [], 'qr': None, 'status': 'new',
                     'pos': pos.copy(), 'nrm': nrm.copy()}
                self.candidates.append(c)
            c['obs'].append((dist, pos.copy(), nrm.copy()))
            c['obs'] = sorted(c['obs'], key=lambda o: o[0])[:12] if len(c['obs']) > 60 else c['obs']
            best = sorted(c['obs'], key=lambda o: o[0])[:10]       # closest views are most accurate
            c['pos'] = np.median([o[1] for o in best], axis=0)
            n = np.median([o[2] for o in best], axis=0)
            c['nrm'] = n / (np.linalg.norm(n) or 1.0)
            return c

    def detect_pillars(self, img, Rmc, tmc):
        scan = self.scan
        if scan is None:
            return
        Tl = self.lookup('map', 'lidar_link')                      # fixed name (see above)
        if Tl is None:
            return
        Rl, tl = Tl
        lidar_yaw = math.atan2(Rl[1, 0], Rl[0, 0])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        H, W = img.shape[:2]
        fx, cx = self.K[0, 0], self.K[0, 2]
        ranges = np.array(scan.ranges)
        for colour, bands in PILLAR_HSV.items():
            mask = np.zeros((H, W), np.uint8)
            for lo, hi in bands:
                mask |= cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cs:
                continue
            c = max(cs, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            if cv2.contourArea(c) < 250 or w * h > 0.5 * W * H:
                continue
            if not (h >= 1.2 * w or y <= 1 or y + h >= H - 1):      # pillars are tall
                continue
            if x <= 1 or x + w >= W - 1:                            # cut at the side: bearing unreliable
                continue
            d_cam = np.array([(x + w / 2.0 - cx) / fx, 0.0, 1.0])
            d_map = Rmc @ d_cam
            yaw = math.atan2(d_map[1], d_map[0])
            idx = (wrap(yaw - lidar_yaw) - scan.angle_min) / scan.angle_increment
            k = int(round(idx))
            win = int(math.radians(3) / scan.angle_increment)
            idxs = [(k + j) % len(ranges) for j in range(-win, win + 1)]
            r = ranges[idxs]
            r = r[np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, 9.0))]
            if r.size == 0:
                continue
            rng = float(r.min()) + 0.2                              # +radius -> pillar centre
            p = np.array([tl[0] + rng * math.cos(yaw), tl[1] + rng * math.sin(yaw)])
            with self.lock:
                self.pillar_obs[colour].append(p)
                self.pillar_obs[colour] = self.pillar_obs[colour][-400:]

    def pillar_pos(self, colour):
        with self.lock:
            pts = np.array(self.pillar_obs[colour])
        if len(pts) < 8:
            return None
        med = np.median(pts, axis=0)
        for _ in range(3):
            keep = np.linalg.norm(pts - med, axis=1) < 0.4
            if keep.sum() >= 5:
                med = np.median(pts[keep], axis=0)
        return med

    # ------------------------------------------------------------------ map helpers
    def nearest_free(self, x, y, max_r=3.0):
        if self.blocked is None:
            return x, y
        res, ox, oy, w, h = self.map_info
        ix, iy = int((x - ox) / res), int((y - oy) / res)
        m = int(max_r / res)
        x0, x1, y0, y1 = max(ix - m, 0), min(ix + m + 1, w), max(iy - m, 0), min(iy + m + 1, h)
        if x0 >= x1 or y0 >= y1:
            return x, y
        free = np.argwhere(~self.blocked[y0:y1, x0:x1])
        if len(free) == 0:
            return x, y
        free = free + [y0, x0]
        d = np.hypot(free[:, 0] - iy, free[:, 1] - ix)
        j = int(np.argmin(d))
        return ox + (free[j, 1] + 0.5) * res, oy + (free[j, 0] + 0.5) * res

    def search_waypoints(self, center):
        cx, cy = center
        pts = [(cx, cy)] + [(cx + self.ring_r * math.cos(a), cy + self.ring_r * math.sin(a))
                            for a in np.linspace(0, 2 * math.pi, 7)[:-1]]
        out = []
        for px, py in pts:
            fx_, fy_ = self.nearest_free(px, py)
            if all(math.hypot(fx_ - a, fy_ - b) > 0.5 for a, b, _ in out):
                out.append((fx_, fy_, math.atan2(cy - fy_, cx - fx_)))
        return out

    def explore_waypoints(self):
        if self.blocked is None:
            return []
        res, ox, oy, w, h = self.map_info
        step = max(1, int(2.0 / res))
        pts = [(ox + (ix + .5) * res, oy + (iy + .5) * res, 0.0)
               for iy in range(0, h, step) for ix in range(0, w, step) if not self.blocked[iy, ix]]
        pose = self.robot_pose()
        cur = (pose[0], pose[1]) if pose else (0.0, 0.0)
        ordered = []
        while pts:                                              # greedy nearest-neighbour tour
            j = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - cur[0], pts[i][1] - cur[1]))
            ordered.append(pts.pop(j))
            cur = ordered[-1][:2]
        return ordered

    # ------------------------------------------------------------------ motion
    def set_status(self, s):
        self.status_pub.publish(String(data=s))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def go_to(self, x, y, yaw=0.0, interrupt=None, timeout=150.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(yaw)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q
        self.set_status('MOVING')
        fut = self.nav.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done():
            if (interrupt and interrupt()) or time.time() - t0 > timeout:
                gh.cancel_goal_async()
                time.sleep(0.5)
                self.stop()
                return False
            time.sleep(0.1)
        return res.result().status == GoalStatus.STATUS_SUCCEEDED

    def spin(self, interrupt=None, rate=0.6):
        """Rotate one full turn in place, looking around."""
        self.set_status('SEARCHING')
        pose = self.robot_pose()
        if pose is None:
            return
        prev, total, t0 = pose[2], 0.0, time.time()
        while total < 2 * math.pi + 0.1 and time.time() - t0 < 60:
            if interrupt and interrupt():
                break
            cmd = Twist()
            cmd.angular.z = rate
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
            pose = self.robot_pose()
            if pose:
                total += wrap(pose[2] - prev)
                prev = pose[2]
        self.stop()

    # ------------------------------------------------------------------ board logic
    def ready(self):
        with self.lock:
            return any(c['id'] == self.expected_id and c['status'] != 'rejected'
                       for c in self.candidates)

    def valid_clue(self, text, prev):
        p = (text or '').split(':', 3)
        if len(p) < 4 or p[0] != 'HUNT':
            return False
        return p[1] == str(self.expected_id) and p[2] == token_of(prev)

    def try_candidates(self, prev):
        while rclpy.ok():
            with self.lock:
                pool = [c for c in self.candidates
                        if c['id'] == self.expected_id and c['status'] != 'rejected']
            if not pool:
                return None
            pose = self.robot_pose() or (0, 0, 0)
            c = min(pool, key=lambda c: np.hypot(c['pos'][0] - pose[0], c['pos'][1] - pose[1]))
            if c['qr'] and not self.valid_clue(c['qr'], prev):   # decoded from afar and wrong
                self.get_logger().info(f"rejecting look-alike id {c['id']}: {c['qr']}")
                c['status'] = 'rejected'
                continue
            if self.approach_and_read(c, prev):
                return c
            c['status'] = 'rejected'
        return None

    def approach_and_read(self, c, prev):
        self.get_logger().info(f"approaching candidate id {c['id']} at {c['pos'][:2].round(2)}")
        for dist in (self.standoff, 0.9, 1.6):
            pos, nrm = c['pos'].copy(), c['nrm'].copy()
            gx, gy = self.nearest_free(pos[0] + dist * nrm[0], pos[1] + dist * nrm[1], 1.0)
            self.go_to(gx, gy, math.atan2(pos[1] - gy, pos[0] - gx))
            self.set_status('READING')
            c['obs'] = []                                        # restart the estimate from close range
            t0 = time.time()
            while time.time() - t0 < 4.0 and not c['qr']:
                time.sleep(0.1)
            if c['qr']:
                if self.valid_clue(c['qr'], prev):
                    time.sleep(1.0)                              # a few more close-range views
                    return True
                self.get_logger().info(f"rejecting look-alike id {c['id']}: {c['qr']}")
                return False
        return False

    def find_board(self, prev, center):
        def sweep(wps):
            for wx, wy, wyaw in wps:
                if wx is not None and not self.ready():
                    self.go_to(wx, wy, wyaw, interrupt=self.ready)
                if not self.ready():
                    self.spin(interrupt=self.ready)
                b = self.try_candidates(prev)
                if b:
                    return b
            return None

        b = self.try_candidates(prev)
        if b:
            return b
        wps = [(None, None, 0.0)] if center is None else self.search_waypoints(center)
        b = sweep(wps)
        if b:
            return b
        self.get_logger().warn('not found near the hint - widening the search')
        return sweep(self.explore_waypoints())

    def ensure_pillars(self, colours):
        def known():
            return all(self.pillar_pos(c) is not None for c in colours)
        if not known():
            self.spin(interrupt=known)
        for wx, wy, wyaw in self.explore_waypoints():
            if known():
                break
            self.go_to(wx, wy, wyaw, interrupt=known)
            self.spin(interrupt=known)
        return {c: self.pillar_pos(c) for c in colours}

    # ------------------------------------------------------------------ mission
    def wait_ready(self):
        self.get_logger().info('waiting for Nav2, map, camera ...')
        self.nav.wait_for_server(timeout_sec=120.0)
        t0 = time.time()
        while rclpy.ok() and (self.K is None or self.blocked is None or self.robot_pose() is None):
            if time.time() - t0 > 3 and int(time.time() - t0) % 2 == 0:   # AMCL needs a start pose
                m = PoseWithCovarianceStamped()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.pose.pose.orientation.w = 1.0
                m.pose.covariance[0] = m.pose.covariance[7] = 0.05
                m.pose.covariance[35] = 0.05
                self.init_pub.publish(m)
            time.sleep(0.5)

    def mission(self):
        self.wait_ready()
        prev, n, center = 'START', 1, None
        while rclpy.ok():
            self.expected_id = n
            board = self.find_board(prev, center)
            if board is None:
                self.get_logger().error(f'could not find board {n}')
                break
            text = board['qr']
            self.clue_pub.publish(String(data=text))
            self.board_pub.publish(String(data=f"{n} {board['pos'][0]:.3f} {board['pos'][1]:.3f}"))
            self.get_logger().info(f'valid clue {n}: {text}')
            prev = text
            words = text.split(':', 3)[3].split()
            treasure = words[0] == 'TREASURE'
            if treasure:
                words = words[1:]
            op, args = words[0], words[1:]
            pos = board['pos'][:2]
            nx, ny = board['nrm'][0], board['nrm'][1]
            nn = math.hypot(nx, ny) or 1.0
            nx, ny = nx / nn, ny / nn                            # board +X (xy);  board +Y = (-ny, nx)
            if op == 'GOTO':
                center = (float(args[0]), float(args[1]))
            elif op == 'REL':
                a, b = float(args[0]), float(args[1])
                p = pos + a * np.array([nx, ny]) + b * np.array([-ny, nx])
                if treasure:
                    self.drive_to_treasure(p)
                    return
                center = (float(p[0]), float(p[1]))
            elif op == 'PILLAR':
                center = tuple(self.ensure_pillars([args[0]])[args[0]])
            elif op == 'BETWEEN':
                a, b, f = args[0], args[1], float(args[2])
                P = self.ensure_pillars([a, b])
                center = tuple(P[a] + f * (P[b] - P[a]))
            else:
                self.get_logger().error(f'unknown command {op}')
                break
            n += 1

    def drive_to_treasure(self, p):
        self.get_logger().info(f'treasure at {p.round(2)} - driving there')
        gx, gy = self.nearest_free(p[0], p[1], 0.3)
        self.go_to(gx, gy, 0.0)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(p[0]), float(p[1])
        msg.pose.orientation.w = 1.0
        self.treasure_pub.publish(msg)
        self.set_status('DONE')
        self.get_logger().info('DONE')


def main():
    rclpy.init()
    node = HuntNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=node.mission, daemon=True).start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
