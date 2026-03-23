# TidyBot — Drift Take-Home Simulation

A complete ROS 2 + Gazebo Classic simulation of a home-tidying robot.
The robot navigates a two-room home, locates scattered objects, picks them
up, and returns them to a collection box.

---

## Environment

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Gazebo Classic 11 |
| Build | colcon |

---

## Quick-start (single command)

```bash
# 1. Install system dependencies (only needed once)
sudo apt-get install -y gazebo gazebo-plugin-base \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins ros-humble-gazebo-msgs \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Build
cd ~/drift-takehome
colcon build --symlink-install
source install/setup.bash

# 4. Launch everything
ros2 launch tidybot_gazebo tidybot_full.launch.py
```

Gazebo opens, TidyBot spawns, and after ~8 seconds the navigation, arm,
and gripper nodes start autonomously. The robot follows a collision-free
waypoint tour through both rooms and returns to the start position.

---

## What you will see

1. **Gazebo** opens with the two-room home world.
2. TidyBot spawns in **Room 1 (Living Room)** at position `(1, 0)`.
3. The green collection box is visible in the north-west of Room 1.
4. The robot follows 40 collision-free waypoints: sweeps Room 1 →
   passes through the doorway → sweeps Room 2 (Bedroom) → returns
   to the start position in Room 1.
5. On completion an **exploration path map** is saved to
   `~/exploration_path.png` showing planned vs. actual path.
6. Progress is logged to the terminal and `/task_log` throughout.

---

## Observing outputs

| Topic | Type | Content |
|-------|------|---------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/scan` | `sensor_msgs/LaserScan` | 360° LiDAR |
| `/camera/image_raw` | `sensor_msgs/Image` | Forward camera |
| `/imu` | `sensor_msgs/Imu` | IMU data |
| `/task_log` | `std_msgs/String` | Per-state progress log |
| `/joint_states` | `sensor_msgs/JointState` | Arm joint angles |

Monitor live:
```bash
# Rooms visited + distance logged to terminal and /task_log
ros2 topic echo /task_log

# Odometry
ros2 topic echo /odom

# Sensor check
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic hz /imu
```

---

## Package structure

```
src/
├── tidybot_description/       # Robot URDF/xacro
│   ├── urdf/tidybot.urdf.xacro
│   └── launch/display.launch.py
├── tidybot_gazebo/            # World + launch
│   ├── worlds/home.world
│   └── launch/
│       ├── tidybot_world.launch.py   # world + spawn only
│       └── tidybot_full.launch.py    # ← single command
└── tidybot_navigation/        # Behaviour nodes
    └── tidybot_navigation/
        ├── navigator.py       # waypoint-following explorer
        ├── arm_controller.py  # arm trajectory control
        └── gripper.py         # vacuum gripper (grab/release)
```

---

## Robot specification check

| Spec item | Implementation |
|-----------|---------------|
| 4-wheel skid-steer | `wheel_front_left/right`, `wheel_rear_left/right` — all `continuous`, driven via `libgazebo_ros_planar_move` |
| Base width 0.4–0.6 m | 0.55 m wide, 0.50 m long |
| Vertical torso | `torso_link` fixed to base, 0.55 m tall |
| 3-DOF right arm | `shoulder_right_joint` / `elbow_right_joint` / `wrist_right_joint` (all revolute) |
| Gripper / end-effector | `gripper_right` — flat scoop + vacuum gripper plugin |
| Cosmetic left arm | `upper_arm_left` + `forearm_left` + `hand_left` (fixed joints) |
| LED face panel | `face_link` — flat cyan box on torso front |
| Torso camera | `camera_link` on torso at 60 % height, publishes `/camera/image_raw` |
| Additional sensor | 360° LiDAR (`lidar_link`, top of torso) + IMU (`imu_link`, base) |
| Physically plausible inertials | box/cylinder formulas — no identity matrices |

---

## Home world check

| Spec item | Implementation |
|-----------|---------------|
| Two rooms | Living Room (~6.2×6.2 m) + Bedroom (~5.2×6.2 m) |
| Doorway | 1.6 m wide × 2.0 m tall in shared wall (fits 0.5 m robot) |
| Walls with collision | 8 wall segments, each 0.2 m thick, 2.5 m tall |
| Table + chairs | Dining table + 2 chairs in Room 1; desk + chair in Room 2 |
| Additional furniture | Couch, bookshelf (Room 1); bed, nightstand, wardrobe (Room 2) |
| 5+ small objects | 6 objects (2 red cubes, 1 blue cube, 1 yellow cube, 1 green cube, 1 orange cube) |
| Collection box | Open-top green box at `(0.5, 2.0)` near robot start |
| Ground friction | µ = 0.9 on ground plane |
| Physics config | ODE, step=0.001 s, gravity=-9.81 |

---

## Task flow

The navigator runs a three-phase state machine:

```
WAIT (for sensors) → EXPLORE (follow 40 waypoints through both rooms) → DONE
```

The robot sweeps Room 1, transits through the doorway, sweeps Room 2,
then returns to its start position. Odometry stats and room-visit status
are logged to the `/task_log` topic and the terminal throughout.
On completion an exploration path map is saved to `~/exploration_path.png`.

## Demo video

> *(screen recording — record with `kazam` or `simplescreenrecorder` while simulation runs)*

---

## Dependencies

All ROS 2 Humble + system packages:

```
gazebo
gazebo-plugin-base
ros-humble-gazebo-ros-pkgs
ros-humble-gazebo-ros
ros-humble-gazebo-plugins
ros-humble-gazebo-msgs
ros-humble-xacro
ros-humble-joint-state-publisher
ros-humble-joint-state-publisher-gui
ros-humble-robot-state-publisher   (installed with ros-humble-desktop)
ros-humble-tf2-ros                 (installed with ros-humble-desktop)
```

Python packages beyond ROS 2 defaults: **none**.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Gazebo hangs on start | Run `killall gzserver gzclient` then retry |
| Robot falls through floor | Verify `gazebo-plugin-base` is installed |
| `/scan` not publishing | Ensure `ros-humble-gazebo-plugins` is installed |
| Arm doesn't move | Check `/set_joint_trajectory` topic is alive: `ros2 topic list` |
| Path map not saved | Install `python3-matplotlib`: `pip3 install matplotlib` |
