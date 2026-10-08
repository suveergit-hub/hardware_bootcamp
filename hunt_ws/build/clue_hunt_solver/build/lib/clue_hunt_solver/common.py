"""Shared helpers: image conversion, ArUco detection, pose maths."""
import math

import cv2
import numpy as np

# Board frame (+X out of the face, +Y reader's right, +Z up) expressed in the
# OpenCV marker frame (x right, y up, z out of the face). Columns = board axes.
R_MB = np.array([[0.0, 1.0, 0.0],
                 [0.0, 0.0, 1.0],
                 [1.0, 0.0, 0.0]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def image_to_bgr(msg):
    """sensor_msgs/Image -> BGR numpy array (no cv_bridge needed)."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('rgba8', 'bgra8'):
        img = buf.reshape(h, msg.step)[:, :w * 4].reshape(h, w, 4)
        code = cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    if msg.encoding == 'mono8':
        img = buf.reshape(h, msg.step)[:, :w]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError('unsupported encoding ' + msg.encoding)


class ArucoDetector:
    """Works with both the old (<=4.6) and new (>=4.7) OpenCV ArUco API."""

    def __init__(self, dictionary=cv2.aruco.DICT_4X4_50):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            d = cv2.aruco.getPredefinedDictionary(dictionary)
            self._det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
            self._old = None
        else:
            self._dict = cv2.aruco.Dictionary_get(dictionary)
            self._params = cv2.aruco.DetectorParameters_create()
            self._old = True

    def detect(self, gray):
        if self._old:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        else:
            corners, ids, _ = self._det.detectMarkers(gray)
        if ids is None:
            return [], []
        return [c.reshape(4, 2) for c in corners], [int(i) for i in ids.ravel()]


def marker_pose(corners, size, K, D):
    """solvePnP for one square marker. Returns (rvec, tvec) or None.

    IPPE gives two candidate poses (planar ambiguity) and can fail for a perfectly
    frontal view, so we also try the iterative solver and keep the lowest-error pose.
    """
    s = size / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    best = None
    for flag in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
        try:
            n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, img, K, D, flags=flag)
        except cv2.error:
            continue
        for rv, tv, e in zip(rvs, tvs, errs):
            e = float(np.ravel(e)[0])
            if tv[2] > 0 and (best is None or e < best[0]):
                best = (e, rv, tv)
    if best is None:
        return None
    return best[1].reshape(3), best[2].reshape(3)


def decode_qr_near_marker(img, rvec, tvec, K, D, qr_detector,
                          offset=0.30, half=0.19):
    """Crop the area where the QR should be (right of the marker) and decode it."""
    Rcm, _ = cv2.Rodrigues(rvec)
    pts_board = np.array([[0, offset - half, -half], [0, offset + half, -half],
                          [0, offset + half, half], [0, offset - half, half]], dtype=np.float64)
    pts_marker = (R_MB @ pts_board.T).T                     # board coords -> marker coords
    cam = (Rcm @ pts_marker.T).T + tvec
    if np.any(cam[:, 2] < 0.1):
        return None
    px, _ = cv2.projectPoints(pts_marker, rvec, tvec.reshape(3, 1), K, D)
    px = px.reshape(-1, 2)
    pad = 12
    x0, y0 = np.floor(px.min(axis=0)).astype(int) - pad
    x1, y1 = np.ceil(px.max(axis=0)).astype(int) + pad
    H, W = img.shape[:2]
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None
    crop = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    for scale in (1.0, 2.0, 3.0):
        c = crop if scale == 1.0 else cv2.resize(crop, None, fx=scale, fy=scale,
                                                  interpolation=cv2.INTER_CUBIC)
        for variant in (c, cv2.createCLAHE(2.0, (4, 4)).apply(c)):
            text, _, _ = qr_detector.detectAndDecode(variant)
            if text:
                return text
    return None
