# Weekly Progress: G1 Warehouse Navigation Stack
**March 27-30, 2026**

---

## Demo: Live Robot Control in Simulated Warehouse

**What you are seeing:** A Unitree G1 humanoid robot walking in an NVIDIA warehouse environment, controlled in real-time from a laptop over the network using a virtual joystick. Camera feed, lidar point cloud, and SLAM odometry stream to Lichtblick (open-source Foxglove fork) over Tailscale.

### Architecture

```
Mac (Lichtblick)  <--WebSocket-->  Docker Container  <--DDS-->  Isaac Sim
   Camera view                      Foxglove bridge              G1 robot
   3D lidar view                    Sensor bridges               Trained RL policy
   Virtual joystick                  FAST-LIO SLAM               RTX lidar (360)
   SLAM trajectory                   Waypoint follower            Head camera
                                     unitree_ros2                 IMU
```

---

## What was accomplished

### 1. Policy deployment in unitree_sim_isaaclab

- Trained locomotion policy now runs inside `unitree_sim_isaaclab` using the v2 custom action provider
- The v2 provider uses Isaac Lab's own `ObservationManager` and `ActionManager`, guaranteeing identical behavior to training
- Robot walks stably with arm swing in the warehouse
- Created `Isaac-Locomotion-G129-Warehouse` task with the exact training robot config

### 2. Robot USD with baked-in sensors

- Converted training URDF to USD, saved at `assets/robots/g1-29dof-locomotion/`
- Baked RTX lidar (`OmniLidar` prim) into the robot USD under `torso_link/Lidar`
- Head camera defined in task config via `CameraCfg`
- Robot config `G129_CFG_LOCOMOTION` loads USD directly (no URDF conversion at runtime)

### 3. DDS control pipeline

- Velocity commands sent over Unitree's CycloneDDS protocol (same as real robot)
- `send_commands_keyboard.py` for terminal control (WASD)
- Lichtblick virtual joystick (`foxglove-nipple` extension) for remote analog control
- `cmd_vel_to_dds.py` bridge with 50 Hz publishing and command smoothing

### 4. Docker container (Isaac ROS G1)

- Built `isaac-ros-g1` Docker image based on NVIDIA Isaac ROS (Jazzy)
- CycloneDDS middleware (matches Unitree's protocol)
- `unitree_ros2` message definitions built at image build time
- FAST-LIO2 compiled from source (patched Livox SDK2 for GCC 13 / Ubuntu 24.04)
- KISS-ICP built from source
- All in `zeulewan/isaac-ros-dev` repo under `g1/`

### 5. Sensor streaming

| Sensor | Source | Bridge | ROS 2 Topic | Rate |
|--------|--------|--------|-------------|------|
| Head camera | Isaac Lab CameraCfg | camera_bridge.py (ZMQ) | /head_camera/image/compressed | ~11 Hz |
| RTX lidar | Baked OmniLidar prim | lidar_bridge.py (ZMQ) | /lidar/points | ~11 Hz |
| IMU | robot.data (sim) | imu_bridge.py (ZMQ) | /imu/data | ~23 Hz |
| 2D scan | pointcloud_to_laserscan | ROS 2 node | /scan | ~11 Hz |

### 6. SLAM (FAST-LIO2)

- Built FAST-LIO2 for ROS 2 Jazzy (patched Livox SDK2 `#include <cstdint>` for GCC 13)
- Configured with `lidar_type: 4` (simulation mode, no per-point timestamps needed)
- Publishes `/Odometry` (pose), `/Laser_map` (accumulated map), `/cloud_registered` (registered scans), `/path` (trajectory)
- TF chain: `camera_init -> body -> lidar`, `camera_init -> body -> head_camera`
- Odom TF bridge republishes as `map -> odom -> base_link` for Nav2 compatibility

### 7. Waypoint follower

- Simple proportional waypoint follower (~100 lines, based on g1pilot's approach)
- Receives `/goal_pose` from Lichtblick, reads `/Odometry` from FAST-LIO
- Computes body-frame velocity commands with proportional control
- Sends `/cmd_vel` to walk toward goal
- Tested: robot walks toward commanded waypoints
- Remaining: obstacle avoidance, FAST-LIO drift correction

### 8. Lichtblick (open-source Foxglove)

- Installed on Mac as desktop app
- Virtual joystick extension (`foxglove-nipple`) for analog control
- Layout auto-load from `~/.lichtblick-suite/layouts/`
- Deep link support for auto-connect: `lichtblick://open?ds=foxglove-websocket&ds.url=...`
- Topics visible: camera, lidar, SLAM map, trajectory, odometry

### 9. Infrastructure

- Full restart script: `~/GIT/unitree_sim_isaaclab/scripts/restart_all.sh`
- Mac restart shortcut: `~/Desktop/restart-sim.sh`
- Tailscale direct connection (fixed Kingston UniFi NAT for UDP hole punching)
- Docker container auto-starts bridges via entrypoint scripts

---

## What matches the real robot

| Component | Simulation | Real G1 |
|-----------|-----------|---------|
| Communication | CycloneDDS | CycloneDDS |
| Policy format | JIT/ONNX | ONNX (Jetson Orin) |
| unitree_ros2 | Same package | Same package |
| Docker container | Isaac ROS (same image base) | Isaac ROS (same image base) |
| Lichtblick | Same frontend | Same frontend |
| DDS topic names | Identical | Identical |
| FAST-LIO | Same package | Same package |
| Waypoint follower | Same code | Same code |

---

## Known issues

1. **FAST-LIO drift** -- z-axis drifts over time, IMU integration accumulates error
2. **No obstacle avoidance** -- waypoint follower drives straight to goal, walks into walls
3. **Container CPU load** -- FAST-LIO + bridges + Foxglove compete for resources, causes camera lag
4. **RTX lidar partial scans** -- per-frame annotator gives wedge scans not full 360, confuses KISS-ICP (FAST-LIO handles it better)

## Next steps

1. **Ground truth odometry for demo** -- publish sim's exact robot pose as `/odom`, skip FAST-LIO for demo performance
2. **Obstacle avoidance** -- check `/scan` for obstacles ahead before sending velocity
3. **Nav2 integration** -- or keep the simple waypoint follower with obstacle checking
4. **Sim-to-real** -- policy distillation (teacher-student) for Jetson Orin deployment

---

## Repositories

| Repo | Purpose |
|------|---------|
| `unitree_sim_isaaclab` | Sim environment + custom action provider + locomotion task |
| `unitree_rl_lab` | RL training + play scripts + play_warehouse.py |
| `isaac-ros-dev` | Docker configs: carter/ and g1/ with Dockerfiles, bridges, nav scripts |
| `robot-docs` | Documentation site |
| `unitree_ros` | Robot URDF descriptions |
