# Isaac Sim & Isaac Lab

Isaac Sim 5.1 and Isaac Lab setup, ROS 2 bridge, and troubleshooting.

## Current Installation

| | |
|---|---|
| **Version** | 5.1.0 (pip) |
| **Conda Env** | `isaaclab` (Python 3.11) |
| **Launch (GUI)** | `conda activate isaaclab && isaacsim` |
| **Launch (headless)** | `conda activate isaaclab && isaacsim isaacsim.exp.full.kit --/app/window/enabled=false --/app/livestream/enabled=false` |

---

## Quick Start

```bash
# GUI mode (requires desktop session via Sunshine/Moonlight)
conda activate isaaclab
isaacsim

# Headless mode (no display needed)
conda activate isaaclab
isaacsim isaacsim.exp.full.kit --/app/window/enabled=false --/app/livestream/enabled=false
```

---

## Headless Launch Script

The `headless-sample-scene.sh` script launches Isaac Sim headless with the Carter warehouse scene, performance optimizations, OmniGraph topic remap (`/cmd_vel` -> `/cmd_vel_raw`), TF publisher disable, and camera culling.

### Starting

```bash
nohup bash ~/Documents/isaac-sim-scripts/headless-sample-scene.sh \
  > /tmp/isaac-sim-headless.log 2>&1 &
```

### Stopping

!!! warning "Killing the bash script does NOT kill Isaac Sim"
    The script uses a heredoc (`python3 - << 'PYEOF'`). Killing the bash wrapper orphans the python child, which keeps running and holding GPU memory. You must kill the python process directly.

```bash
# 1. Find the actual python process (NOT the bash wrapper)
nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv,noheader

# 2. Kill the python3 process by PID
kill <python3_pid>

# 3. Verify GPU memory is freed
nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv,noheader
```

If multiple orphan instances accumulated (each using ~3-5 GB VRAM), kill them all:

```bash
# Kill ALL Isaac Sim python processes at once
nvidia-smi --query-compute-apps=pid,name --format=csv,noheader \
  | grep python3 | cut -d',' -f1 | xargs kill
```

### Checking logs

Python `print()` output does **not** go to `/tmp/isaac-sim-headless.log` — Kit intercepts stdout. Check the Kit log instead:

```bash
# Find the latest Kit log
ls -t ~/miniconda3/envs/isaaclab/lib/python3.11/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim\ Python/5.1/kit_*.log | head -1

# Search for our custom tags
grep "\[py stdout\]" <kit_log_path> | grep -E "\[CAM\]|\[CMD\]|\[TF\]|\[PHYS\]"
```

---

## Installation Methods

### Via pip (Current)

```bash
# Requires Python 3.11, GLIBC 2.35+
conda activate isaaclab
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

### Standalone (removed Feb 2026)

Standalone install at `~/isaac-sim/` removed to save 17 GB. Pip version has the same features.

---

## Isaac Lab

Robot learning framework built on Isaac Sim.

### Installation (after Isaac Sim works)

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install
```

### Isaac Sim vs Isaac Lab

| | Isaac Sim | Isaac Lab |
|---|-----------|-----------|
| **Purpose** | Full simulation platform + GUI | Robot learning framework |
| **Use** | Design environments, test robots | Train RL policies at scale |
| **Mode** | Interactive GUI | Headless training |
| **Install** | pip in conda env | Separate from source |

---

## ROS 2 Bridge (bundled in Isaac Sim)

- ROS 2 Jazzy libs bundled in `isaacsim.ros2.bridge` extension (Python 3.11)
- Requires env vars to load correctly -- use `~/bin/isaacsim-ros` wrapper
- **No system ROS 2 on host** -- conflicts (Python 3.12 vs 3.11 ABI mismatch)
- DDS handles communication between Isaac Sim and external ROS 2 nodes regardless of Python version

---

## Foxglove Visualization

- foxglove-bridge on port 8765 (container, `--network host` so accessible on localhost)
- Connect from any browser: `ws://workstation:8765` or `ws://localhost:8765`
- Raw images too large for remote viewing -- use compressed video topic instead
- H.264 NVENC republisher encodes on GPU (~50KB/frame vs 2.7MB raw)
- H.265 NVENC does not work with Foxglove (browser can't decode keyframes)
- NVENC requires `gop_size` > 1 (B-frame constraint). Use `gop_size:=10`

### Republisher command

```bash
ros2 run image_transport republish raw foxglove --ros-args \
  -r in:=/front_stereo_camera/left/image_rect_color \
  -r out/foxglove:=/front_stereo_camera/left/compressed_video \
  -p out.foxglove.encoder:=h264_nvenc \
  -p out.foxglove.gop_size:=10 \
  -p out.foxglove.bit_rate:=5000000 \
  -p out.foxglove.qmax:=10 \
  -p out.foxglove.encoder_av_options:='forced-idr:1,preset:p1,tune:ll'
```

---

## ROS 2 Topics (Carter Warehouse Scene)

The headless sample scene (`carter_warehouse_apriltags_worker.usd`) publishes these topics:

### Sensors

| Topic | Type | Description |
|---|---|---|
| `/front_stereo_camera/left/image_rect_color` | `sensor_msgs/Image` | Left stereo camera (rectified) |
| `/front_stereo_camera/right/image_rect_color` | `sensor_msgs/Image` | Right stereo camera (rectified) |
| `/front_stereo_camera/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera calibration |
| `/front_stereo_camera/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera calibration |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 3D lidar point cloud |
| `/chassis/imu` | `sensor_msgs/Imu` | Chassis IMU |
| `/front_stereo_imu/imu` | `sensor_msgs/Imu` | Front stereo camera IMU |
| `/back_stereo_imu/imu` | `sensor_msgs/Imu` | Back stereo camera IMU |
| `/left_stereo_imu/imu` | `sensor_msgs/Imu` | Left stereo camera IMU |
| `/right_stereo_imu/imu` | `sensor_msgs/Imu` | Right stereo camera IMU |

### Robot State

| Topic | Type | Description |
|---|---|---|
| `/chassis/odom` | `nav_msgs/Odometry` | Robot position and velocity (ground truth from sim) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree — spatial relationships between all frames |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |

### Control

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from Nav2/teleop |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Same as `/cmd_vel` (pass-through relay) — actual input to Isaac Sim's OmniGraph differential drive |

### Added by Container

| Topic | Type | Description |
|---|---|---|
| `/front_stereo_camera/left/compressed_video` | `foxglove_msgs/CompressedVideo` | H.264 NVENC compressed stream (from republisher) |

!!! note "Rectified images"
    "Rectified" means lens distortion is removed and stereo pairs are aligned so left/right pixels share the same row — ready for depth computation. In sim it's a formality (virtual cameras have no distortion), but the topic names match real camera pipelines.

---

## TF Tree (Carter Warehouse Scene)

The Nova Carter robot has a large sensor suite. The transform tree shows how every frame relates to `base_link`:

```
odom                              ← world-fixed reference frame
 └── base_link                    ← robot center (ground plane)
      ├── nova_carter             ← robot body
      │    ├── wheel_left
      │    ├── wheel_right
      │    └── caster_frame_base
      │         ├── caster_swivel_left → caster_wheel_left
      │         └── caster_swivel_right → caster_wheel_right
      ├── chassis_imu
      ├── front_3d_lidar
      ├── front/back_2d_lidar
      ├── front_stereo_camera     (left_optical, right_optical, imu)
      ├── left_stereo_camera      (left_optical, right_optical, imu)
      ├── right_stereo_camera     (left_optical, right_optical, imu)
      ├── rear_stereo_camera      (left_optical, right_optical, imu)
      ├── front/left/right/back_fisheye_camera_optical
      ├── front/left/right/back_hawk (stereo pairs)
      └── mount
```

Each transform is a 4x4 homogeneous matrix (rotation + translation). ROS uses the TF tree to convert points between frames — e.g., transforming a lidar point from the `front_3d_lidar` frame to `odom` (world coordinates).

Generate the full tree as a PDF:

```bash
ros2 run tf2_tools view_frames
```

---

## Software Versions (February 2026)

| Component | Version | Notes |
|-----------|---------|-------|
| Isaac Sim | 5.1.0 | Pip install in conda env `isaaclab` |
| Isaac Sim | 6.0 | Early dev preview only |
| Isaac Lab | 2.3.2 | Separate install from source |

---

## Troubleshooting

### Isaac Sim won't start

```bash
# Check DISPLAY variable
echo $DISPLAY  # Should be :0

# Check GPU
nvidia-smi

# Check logs
ls ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit_*.log
```

### Slow first launch

Shader compilation takes 5-10 minutes on first run. Subsequent launches are faster.

### GPU not being used

```bash
# Check GPU memory usage while Isaac Sim runs
watch -n 1 nvidia-smi
```

---

## File Locations

```
~/.nvidia-omniverse/logs/       # Isaac Sim logs
~/.local/share/ov/data/Kit/     # Kit config and layout files
~/IsaacLab/                     # Isaac Lab source
```

---

## References -- The Three Doc Sites

| Site | What it is | Follow for install? |
|------|-----------|----------------------|
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/index.html) | Robot learning framework (includes Isaac Sim pip install) | **YES** -- our Isaac Sim + Isaac Lab install source |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/) | Simulator docs (GUI, sensors, ROS bridge config, extensions) | **NO** for install. Reference only |
| [Isaac ROS](https://nvidia-isaac-ros.github.io/index.html) | GPU-accelerated ROS 2 perception packages | **YES** -- our Isaac ROS Docker install source |

Isaac Lab handles Isaac Sim installation. Isaac Sim docs are reference only -- don't follow their install guides. Isaac ROS uses Docker (no host ROS 2).
