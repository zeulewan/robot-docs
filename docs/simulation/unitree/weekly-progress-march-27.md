# Weekly Progress: G1 Warehouse Navigation Stack
**March 27, 2026**

---

## Demo: Live Robot Control in Simulated Warehouse

**What you are seeing:** A Unitree G1 humanoid robot walking in an NVIDIA warehouse environment, controlled in real-time from a laptop over the network using a virtual joystick.

### Architecture

```
Mac (Lichtblick)  <--WebSocket-->  Docker Container  <--DDS-->  Isaac Sim
   Camera view                      Foxglove bridge              G1 robot
   Lidar view                       Sensor bridges               Trained RL policy
   Virtual joystick                  unitree_ros2                 RTX lidar
                                                                  Head camera
```

### What was accomplished this week

**1. Policy deployment in unitree_sim_isaaclab**

- Trained locomotion policy (7200 iterations) now runs inside `unitree_sim_isaaclab` using a custom action provider
- Previously the robot collapsed immediately due to observation/action pipeline bugs
- Root cause: the custom provider was manually constructing observations instead of using Isaac Lab's built-in managers
- Fix: rewrote the provider to use `env.observation_manager` and `env.action_manager` directly, guaranteeing identical behavior to training
- Robot now walks stably with arm swing in the warehouse

**2. Custom Isaac Lab task**

- Created `Isaac-Locomotion-G129-Warehouse` task with the exact training robot config (URDF, actuators, joint defaults)
- Robot USD includes baked-in RTX lidar sensor on the torso
- Warehouse scene loaded as USD terrain

**3. DDS keyboard and joystick control**

- Velocity commands sent over Unitree's CycloneDDS protocol (same as the real robot)
- `send_commands_keyboard.py` for terminal control (WASD)
- Lichtblick virtual joystick for remote control with smoothed ramping

**4. Docker container (Isaac ROS)**

- Built `isaac-ros-g1` Docker image based on NVIDIA Isaac ROS (Jazzy)
- Switched from FastDDS to CycloneDDS to match Unitree's protocol
- Installed `unitree_ros2` message definitions so ROS 2 sees Unitree DDS topics natively
- Container runs: Foxglove bridge, camera/lidar/IMU bridges, cmd_vel relay

**5. Sensor streaming**

- **Head camera:** RGB frames from Isaac Lab camera sensor, published as compressed JPEG at ~12 Hz via ZMQ bridge to ROS 2
- **RTX lidar:** 360-degree point cloud baked into robot USD, published via ZMQ to ROS 2 PointCloud2
- **IMU:** Angular velocity + linear acceleration from sim, published as sensor_msgs/Imu

**6. Lichtblick (open-source Foxglove)**

- Installed on Mac, connects over Tailscale to workstation
- Layout with: camera view, virtual joystick, 3D lidar panel, robot state
- Virtual joystick extension (`foxglove-nipple`) for analog control
- Auto-load layout from `~/.lichtblick-suite/layouts/`

**7. SLAM (in progress)**

- KISS-ICP lidar odometry installed and tested
- Builds point cloud map as robot drives around
- RTX lidar rate limited to ~1.4 Hz due to render pipeline (needs optimization)
- Isaac Lab RayCaster alternative tested but warehouse collision meshes needed
- Next step: RTAB-Map (apt-installable, lidar+IMU fusion, Nav2 compatible)

### What matches the real robot

| Component | Simulation | Real G1 |
|-----------|-----------|---------|
| Communication | CycloneDDS | CycloneDDS |
| Policy format | JIT/ONNX | ONNX (on Jetson Orin) |
| unitree_ros2 | Same package | Same package |
| Perception container | Isaac ROS Docker | Isaac ROS Docker |
| Foxglove/Lichtblick | Same frontend | Same frontend |
| DDS topic names | Identical | Identical |

### Next steps

1. **Fix lidar rate** -- switch from RTX lidar to RayCaster with collision meshes, or optimize RTX render pipeline
2. **RTAB-Map SLAM** -- apt install, takes PointCloud2 + IMU, outputs occupancy grid for Nav2
3. **Nav2 navigation** -- click a goal in Lichtblick, robot walks there autonomously
4. **Sim-to-real transfer** -- policy distillation (teacher-student) for deployment on Jetson Orin

### Repositories

| Repo | Purpose |
|------|---------|
| `unitree_sim_isaaclab` | Simulation environment + custom action provider |
| `unitree_rl_lab` | RL training + play scripts |
| `isaac-ros-dev` | Docker configs for perception container |
| `robot-docs` | Documentation |
| `unitree_ros` | Robot URDF descriptions |
