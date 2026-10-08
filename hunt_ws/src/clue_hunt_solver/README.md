# clue_hunt_solver — Clue Chain Hunt (leader + follower)

## Setup (once)
```bash
sudo apt install -y ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-tf2-tools \
  ros-humble-xacro ros-humble-robot-state-publisher python3-opencv python3-numpy
mkdir -p ~/hunt_ws/src ~/hunt_ws/maps
cp -r Clue_Chain_Hunt_Bootcamp/clue_hunt_* ~/hunt_ws/src/     # this folder replaces the template clue_hunt_solver
cd ~/hunt_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
```

## Step A — build the map (once, teleop allowed)
```bash
ros2 launch clue_hunt_navigation mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # drive slowly round the whole arena
ros2 run nav2_map_server map_saver_cli -f ~/hunt_ws/maps/arena
cp ~/hunt_ws/maps/arena.* ~/hunt_ws/src/clue_hunt_solver/maps/   # keep the map in your package
```
Close that terminal set (Ctrl+C everything).

## Step B — the autonomous run (3 terminals)
```bash
# 1  simulation (both robots)
ros2 launch clue_hunt_gazebo sim.launch.py
# 2  Nav2 on your saved map
ros2 launch clue_hunt_navigation navigation.launch.py map:=$HOME/hunt_ws/maps/arena.yaml
# 3  your solution (leader + follower)
ros2 launch clue_hunt_solver hunt.launch.py
```
The leader sets its own initial pose at (0,0) (it spawns at the origin). Watch the log for
`valid clue N: ...` lines; the last line is `DONE`.

## Nodes
| node | what it does |
|---|---|
| `hunt_node` | ArUco+QR detection, solvePnP -> map frame, token check, clue parser, search (spin + ring around the hint), Nav2 goals, pillar finder (camera colour + LiDAR range), treasure |
| `follower_node` | detects tag 49, leader position in `follower/odom`, breadcrumb path following, distance keeping |
| `common.py` | image conversion, ArUco wrapper (old+new OpenCV API), pose helpers, QR crop+decode |

Parameters: `standoff` (read distance, 1.2 m), `search_radius` (ring radius, 1.3 m), follower `d_stop`, `v_max`.

## Known limitations
No pre-built-map (live SLAM) mode; no RViz markers. Follower has no obstacle sensing (it relies on following the leader's path).
