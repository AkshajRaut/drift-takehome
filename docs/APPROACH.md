# TidyBot — Approach Document

## 1. Robot Model

### Design Philosophy
I built the robot entirely from scratch in URDF/xacro — no existing models or meshes were used as a base. Every link dimension, mass, and inertia tensor is analytically computed so that when something goes wrong in simulation, I can trace the problem to a specific value and fix it.

### Base and Mobility
The base is a 0.50 × 0.55 × 0.15 m box at 30 kg — heavy enough to resist tipping when the arm extends forward. Four wheels (r = 0.10 m, width = 0.05 m) are placed at ±0.15 m fore/aft and ±0.30 m laterally for a stable footprint. The collision geometry uses a cylinder (0.35 m radius × 0.60 m height, centred at z = 0.25 m) rather than matching the visual box — this produces smoother contact response when brushing past furniture corners. The cylinder is kept short enough that the torso-mounted LiDAR (z ≈ 0.665 m above base_link) sits above it and is not blocked by the robot's own collision.

Mobility uses `libgazebo_ros_planar_move` for direct velocity control (linear.x + angular.z). I initially tried `libgazebo_ros_diff_drive` with all four wheels, but Gazebo Classic 11 only supports exactly one left + one right joint (see Debugging section). Planar move sidesteps per-wheel PID tuning entirely and lets the navigator focus on path-level control.

**Inertials** are computed analytically — box: `I_xx = m/12*(y²+z²)`, cylinder: `I_xx = m/12*(3r²+h²)`. No identity matrices or placeholder values.

### 3-DOF Right Arm
Three revolute joints about the Y-axis (pitch in the XZ plane):

| Joint | Range | Purpose |
|-------|-------|---------|
| Shoulder | −π/2 … π | Swings arm forward/back |
| Elbow | −π … π | Bends forearm |
| Wrist | −π/2 … π/2 | Wrist pitch |

Rest-pose offsets (shoulder −0.5 rad, elbow −2.5 rad) tuck the arm naturally against the torso at startup. The end-effector is a flat 0.12 × 0.10 × 0.02 m scoop with a `libgazebo_ros_vacuum_gripper` plugin — a design that avoids the gap-closing complexity of parallel-jaw grippers.

### Cosmetic Left Arm
Mirrors the right arm geometry with fixed joints set to a relaxed pose (shoulder −0.5 rad, elbow −2.5 rad). Crucially, the left arm has **no `<collision>` elements** — only visual geometry. This was a hard-won lesson (see Bug 7 below).

### Sensors
- **Camera**: 640×480, 80° FOV, 10 Hz, mounted at 60% torso height → `/camera/image_raw`
- **360° LiDAR**: 720 rays, 0.12–8 m range, 10 Hz, torso-mounted → `/scan`
- **IMU**: 100 Hz at base origin → `/imu`

---

## 2. Home World Design

### Layout
Two rooms connected by a 1.6 m wide × 2.0 m tall doorway in a shared wall:

```
┌──────────────────────────┬─────────────────────┐
│  Living Room (6.2×6.2m)  │  Bedroom (5.2×6.2m) │
│                          │                     │
│  [couch]     [shelf]     │     [bed]           │
│        [box]         ┌───┘     [nightstand]    │
│  ★start          door│         [wardrobe]      │
│       [table]        └───┐     [desk] [chair]  │
│    [chair1] [chair2]     │                     │
└──────────────────────────┴─────────────────────┘
```

Furniture placement was intentional — the dining table and chairs create a narrow corridor between the south wall and room centre, forcing the robot to navigate carefully. The bookshelf and couch constrain the north side. In Room 2, the bed dominates the east half, leaving an L-shaped navigable path.

All furniture heights are at least 0.80 m so they intersect the LiDAR scan plane (0.765 m above ground), ensuring the robot detects every obstacle. Heights are varied to look realistic: table 0.80 m, nightstand 0.80 m, bed 0.90 m, desk 0.95 m, chairs/couch 0.85 m, bookshelf 1.80 m, wardrobe 2.00 m.

### Objects and Collection Box
Six 0.08 m coloured cubes (mass = 0.2 kg, µ = 0.6) are scattered across both rooms. The open-top green collection box at (0.5, 2.0) is built from five static SDF panels. The vacuum gripper plugin is configured to exclude furniture and walls from pickup.

### Physics
ODE solver with step = 0.001 s, 200 iterations, gravity = −9.81. Ground friction µ = 0.9 with contact stiffness kp = 100000. These values prevent robot drift while keeping simulation at real-time speed.

---

## 3. Navigation Strategy

### Approach: Pure Waypoint Following
I chose hard-coded waypoints over Nav2 for several reasons:
1. The environment is fully known and static — a planner adds complexity without benefit
2. Nav2 requires 10+ additional packages, a pre-built map, and AMCL tuning

### Control System
The navigator uses a PD heading controller with velocity modulation:

```
desired_yaw = atan2(dy, dx)
heading_error = wrap(desired_yaw - current_yaw)
angular_vel = Kp * error + Kd * d(error)/dt     (clamped to ±1.2 rad/s)
alignment = max(0, cos(min(|error|, π/2)))
linear_vel = MAX_LIN * alignment                 (0 if |error| > 55°)
```

The robot stops rotating in place until roughly aligned, then accelerates proportionally to alignment quality. Final waypoint approach uses distance-based deceleration over 0.6 m.

### Coverage Path
34 waypoints sweep both rooms in a systematic pattern:
1. **Room 1 clockwise**: SW corner → south wall → SE corner → east wall → north corridor (between table and bookshelf) → back south
2. **Doorway transit**: approach → through centre
3. **Room 2 south sweep**: desk area → SE corner → east wall
4. **Room 2 north sweep**: NW → NE → back
5. **Return**: through doorway → south of table → home position

The state machine is simple: **WAIT** (for `/odom` and `/scan`) → **EXPLORE** (follow all waypoints) → **DONE**.

### LiDAR Mapper
A fourth node (`mapper`) builds a 2-D occupancy grid in real time from `/scan` and `/odom`. It uses Bresenham ray tracing with log-odds updates (L_FREE = −0.3, L_OCC = 0.9) on a 320 × 220 cell grid (0.05 m resolution, covering 16 × 11 m). The grid is published at 2 Hz on `/map` as a `nav_msgs/OccupancyGrid`. On exploration finish (or Ctrl-C), the node captures a screenshot of the RViz window and saves it to `output/lidar_map.png`.

### RViz Visualisation
Both `tidybot_world.launch.py` and `tidybot_full.launch.py` launch RViz with a pre-configured top-down orthographic view (`mapping.rviz`). The config displays the robot model, occupancy map, accumulated LiDAR points (Decay Time = 9999 s), and TF frames. RViz launches fullscreen with side panels hidden to maximise the map viewport.

---

## 4. Debugging Challenges

This section covers the real simulation bugs I encountered and how I resolved them. These are the kinds of problems that don't show up until you actually run the simulation.

### Bug 1: diff_drive "Inconsistent number of joints"
Gazebo Classic 11's `libgazebo_ros_diff_drive` only supports exactly 1 left + 1 right joint tag. Listing all 4 wheels caused a silent plugin failure — the robot spawned but couldn't move. **Fix:** Switched to `libgazebo_ros_planar_move` which handles velocity directly without per-wheel configuration.

### Bug 2: Robot invisible in Gazebo
The launch file used `Command(['xacro', ...])` to pass the URDF as a shell argument. URDF XML contains `<`, `>`, `"` which broke shell argument parsing — the robot description was silently mangled. **Fix:** Rewrote the launch to use `OpaqueFunction` + `subprocess.run()` so the URDF string stays in Python and never touches the shell.

### Bug 3: LiDAR detecting own arm at spawn (the sneakiest bug)
The robot immediately entered obstacle avoidance on every launch. LiDAR inspection revealed readings at exactly `range_min = 0.12 m` at ±34° from forward. The cosmetic left arm had `<collision>` geometry, and its resting pose extended through the LiDAR's horizontal scan plane. The LiDAR was detecting its own arm.

**Fix (two-part):**
1. Removed `<collision>` from all cosmetic left arm links — a visual-only arm should not interact with physics or sensors
2. Added a range filter: `min_valid = max(range_min * 1.5, 0.20 m)` to reject returns at the sensor's saturation floor

This is exactly the kind of bug that separates "URDF that renders" from "URDF that simulates." The model looked perfect in RViz but caused phantom obstacles in Gazebo.

### Bug 4: Arm controller spamming trajectories
`self.create_timer(2.5, callback)` in ROS 2 creates a **repeating** timer. The arm controller kept publishing trajectory messages every 2.5 s indefinitely. **Fix:** Cancel the timer inside its own callback to get one-shot behaviour.

### Bug 5: Infinite obstacle avoidance loop (earlier iteration)
An earlier version used timed avoidance (turn for 1.2 s, then declare clear). Without re-checking LiDAR, the robot turned ~55°, drove into the same obstacle, and triggered avoidance again — infinite loop. **Fix:** Rewrote to reactive avoidance (turn until `obstacle_ahead()` returns False). Later simplified further to the current pure-waypoint system with pre-verified collision-free paths.

### Bug 6: Objects unreachable from waypoints
Static analysis revealed 3 of 6 objects were beyond the 1.2 m pickup trigger distance from any waypoint. **Fix:** Redesigned the waypoint path to pass within range of every object while maintaining ≥ 0.75 m clearance from furniture.

---

## 5. Tradeoffs

| Decision | Alternative | Why I chose this |
|----------|-------------|-----------------|
| Planar move plugin | diff_drive / Ackermann | Avoids per-wheel PID tuning; direct velocity control is simpler and more reliable |
| Hard-coded waypoints | Nav2 + costmap | Known static environment; waypoints are deterministic and launch instantly |
| Vacuum gripper + delete/spawn | ros2_control + force-closure | Deterministic object handling; no grasping physics to debug |
| Cylinder base collision | Matching box collision | Smoother contact with furniture corners; prevents getting stuck on edges. Cylinder height kept below LiDAR to avoid self-occlusion |
| Custom Bresenham mapper | slam_toolbox / cartographer | Zero external dependencies; lightweight log-odds grid is sufficient for a known environment |
| RViz screenshot capture | Matplotlib rendering | RViz already renders the accumulated scan + occupancy grid perfectly; capturing the window directly avoids reimplementing the visualisation |
| Box primitives | STL meshes | Faster iteration; the focus is on simulation behaviour, not visual fidelity |

---

## 6. What I Would Improve

1. **Nav2 integration** — Replace waypoints with a costmap + DWB planner so the robot handles unexpected obstacles and dynamic environments.
2. **Real arm control** — Use `gazebo_ros2_control` + `JointTrajectoryController` for physics-accurate arm trajectories with feedback.
3. **Visual object detection** — Replace hardcoded positions with camera-based colour-blob detection (OpenCV HSV thresholding) so the robot discovers objects at runtime.
4. **SLAM** — Generate the map online with `slam_toolbox` instead of assuming a known layout, making the system transferable to new environments.
5. **Custom meshes** — Replace box primitives with STL meshes for a more realistic visual appearance.
6. **Rosbag recording** — Automatically record all sensor topics for offline replay and debugging.

---

## 7. Sources

No existing robot models, URDF files, or mesh assets were used. The entire robot model, world file, and navigation system were built from scratch. The only external dependencies are standard ROS 2 Humble packages and Gazebo Classic 11 plugins.
