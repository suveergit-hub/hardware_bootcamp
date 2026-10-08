# Clue Chain Hunt — Technical Report (max 5 pages)

## 1. System overview
Nodes / topics / launch files (one diagram: Gazebo -> camera, LiDAR -> hunt_node -> Nav2 -> /cmd_vel ; follower camera -> follower_node -> /follower/cmd_vel).

## 2. Clue detection, pose estimation, frames
- ArUco DICT_4X4_50, 0.24 m, solvePnP (IPPE_SQUARE, best of IPPE/iterative by reprojection error).
- Board frame (+X out, +Y reader's right, +Z up) vs OpenCV marker frame: R_MB.
- Chain: marker -> cam_optical_link -> (TF2) map. Median of the 10 closest views.
- QR: crop the region 0.30 m right of the marker (projected with the pose), upscale, decode; token = SHA-1[:4] of previous clue.
- Validation (offline test on the sample board): position error < 2 cm, normal error < 2.5 deg.

## 3. Search / clue-solving strategy
GOTO / REL: go to nearest free cell, spin 360, ring of 6 waypoints (r = 1.3 m). PILLAR / BETWEEN: pillars found at run time (HSV blob + LiDAR range, clustered by median). Decoys ignored by id, look-alikes rejected by token. Fallback: grid exploration.

## 4. Follower design
Tag 49 -> leader centre (0.21 m ahead of tag) -> follower/odom -> breadcrumbs -> follow the path, not the leader; speed ∝ (d − 1.0); stop at 1.0 m; turn-to-reacquire if the tag is lost.

## 5. Results
Screenshots: map, hunt path in RViz, follower view. Table: clues solved, time, min/max follower distance.

## 6. Failure cases
(fill in from your runs: e.g. look-alike read from afar, tag lost on sharp turns, Nav2 goal inside inflation)
