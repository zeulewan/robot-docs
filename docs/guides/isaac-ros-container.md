# Isaac ROS Container

GPU-accelerated ROS 2 perception container for simulation mode. Built from a Dockerfile on top of NVIDIA's base image.

## How It Works

The container is a self-contained ROS 2 environment with GPU access. It receives sensor data from Isaac Sim (running on the host) via DDS, runs GPU perception (SLAM, AprilTag, 3D mapping), and serves visualization over WebSocket.

```mermaid
flowchart LR
    subgraph HOST["Host — Workstation"]
        SIM["Isaac Sim<br/>(simulator)"]
    end
    subgraph CONTAINER["Container — isaac-ros-dev"]
        PERC["GPU Perception<br/>AprilTag, SLAM, nvblox"]
        FOX["Foxglove Bridge :8765<br/>H.264 NVENC republisher"]
    end
    BROWSER["Foxglove Studio<br/>(any browser)"]

    SIM -- "UDP DDS" --> PERC
    PERC -- "UDP DDS" --> SIM
    FOX -- "WebSocket :8765" --> BROWSER
```

The container uses `--network host` so it shares the host's network stack — DDS topics flow freely between Isaac Sim and the container without any port mapping.

---

## Quick Start

```bash
# Start container
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml up -d

# Attach a shell
docker exec -it -u admin isaac_ros_dev_container bash

# Stop
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml down
```

---

## What's in the Base Image

NVIDIA provides a ~38.8 GB base image from NGC:

```
nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_<hash>-amd64
```

This includes:

- **ROS 2 Jazzy** — full desktop install (50+ packages)
- **TensorRT** — GPU-accelerated inference
- **PyTorch** — deep learning framework
- **nav2** — ROS 2 navigation stack
- **rviz2** — 3D visualization
- **foxglove-bridge** — WebSocket bridge for Foxglove Studio
- **slam_toolbox** — SLAM algorithms
- **OpenCV** — computer vision (custom NVIDIA build with CUDA)
- **CUDA 13.0 dev tools** — compiler, libraries, headers
- **VPI, CV-CUDA, Triton** — NVIDIA perception and inference libraries

What's NOT in the base (we add these):

- Isaac ROS perception packages (apriltag, cuVSLAM, nvblox)
- H.264 NVENC video transport for Foxglove
- ffmpeg with NVENC support
- FastDDS configuration for host/container communication

---

## What the Dockerfile Adds

The Dockerfile (`~/workspaces/isaac_ros-dev/Dockerfile`) adds a thin layer on top of the base. The base image is never modified — Docker layers are immutable.

### Packages

Installed from NVIDIA's Isaac ROS apt repository (pre-built .deb packages, not compiled from source):

| Package | What it does |
|---|---|
| `ros-jazzy-isaac-ros-apriltag` | GPU-accelerated AprilTag detection and pose estimation |
| `ros-jazzy-isaac-ros-visual-slam` | cuVSLAM — visual SLAM using stereo cameras on GPU |
| `ros-jazzy-isaac-ros-nvblox` | GPU 3D reconstruction from depth cameras |
| `ros-jazzy-foxglove-compressed-video-transport` | Encodes camera feeds as H.264 for Foxglove |
| `ros-jazzy-ffmpeg-encoder-decoder` | ROS 2 wrapper around ffmpeg for NVENC encoding |
| `ffmpeg` | CLI video tool (used by the encoder) |

After installing, `rm -rf /var/lib/apt/lists/*` deletes the apt package index cache (~50-100 MB) to keep the image smaller. Standard Docker practice.

### FastDDS UDP-Only Transport

**The problem:** ROS 2 uses DDS as its transport layer. On this system, Isaac Sim (host) uses FastDDS, which defaults to shared memory (SHM) when it detects two processes on the same machine. But Docker containers have isolated `/dev/shm` — even with `--ipc host`, FastDDS's SHM implementation fails to map memory across the host/container boundary. The result: `ros2 topic list` works (discovery uses UDP) but `ros2 topic echo` shows nothing (data transfer tries SHM and silently fails).

**The fix:** An XML config at `/etc/fastdds_no_shm.xml` disables SHM and forces all traffic over UDP. The environment variable `FASTRTPS_DEFAULT_PROFILES_FILE` tells FastDDS to load this config.

This env var is set in two places:

- `/etc/profile.d/fastdds-fix.sh` — picked up by login shells (when you `docker exec bash`)
- Explicitly in each entrypoint script — because the container's startup system sources scripts, not login shells, so profile.d doesn't apply

**Sim vs real:** This fix exists for simulation mode where Isaac Sim and the container share the same host. In real robot mode, the G1 uses CycloneDDS over Ethernet — a completely different DDS implementation on a different machine, so this config is irrelevant. However, forcing UDP is harmless in real mode too (UDP works fine over a network), so one image works for both scenarios without needing different configs.

### Entrypoint Scripts

The NVIDIA base image has a startup system: when the container boots, it runs every `.sh` script in `/usr/local/bin/scripts/entrypoint_additions/` in alphabetical order. The `50-` and `60-` prefixes control execution order.

| Script | What it does |
|---|---|
| `50-foxglove-bridge.sh` | Starts foxglove-bridge on port 8765 (WebSocket for Foxglove Studio) |
| `60-h264-republisher.sh` | Starts H.264 NVENC republisher (encodes raw camera topics for remote viewing) |

Both scripts must explicitly set `FASTRTPS_DEFAULT_PROFILES_FILE` because entrypoint scripts are `source`d, not run as login shells.

### Admin User

A non-root `admin` user with passwordless sudo. Used when attaching to the container (`docker exec -it -u admin`).

---

## Running Services

### Foxglove Bridge

| | |
|---|---|
| **Port** | 8765 (WebSocket) |
| **Connect** | `ws://workstation:8765` or `ws://100.101.214.44:8765` (Tailscale) |
| **Auto-starts** | Yes, via entrypoint script |

Opens a WebSocket that Foxglove Studio connects to. Exposes all ROS 2 topics in real-time — camera feeds, SLAM maps, detections, joint states.

### H.264 NVENC Republisher

| | |
|---|---|
| **Subscribes to** | `/front_stereo_camera/left/image_rect_color` (raw) |
| **Publishes** | `/front_stereo_camera/left/compressed_video` (H.264) |
| **Auto-starts** | Yes, via entrypoint script |

Encodes raw camera images using the GPU's hardware H.264 encoder (NVENC). Without this, streaming raw images over the network to Foxglove is too slow.

!!! note
    H.265 does NOT work with Foxglove (browser can't decode keyframes). Use H.264 only.

---

## Files

All in `~/workspaces/isaac_ros-dev/`:

| File | Purpose |
|---|---|
| `Dockerfile` | Builds the image on top of NVIDIA base |
| `docker-compose.yml` | Launches the container with correct flags |
| `fastdds_no_shm.xml` | UDP-only DDS transport config |
| `50-foxglove-bridge.sh` | Foxglove auto-start entrypoint |
| `60-h264-republisher.sh` | H.264 republisher auto-start entrypoint |

### Rebuilding the Image

```bash
cd ~/workspaces/isaac_ros-dev
docker build -t isaac-ros-dev:latest .
```

The base image (38.8 GB) is cached — only the custom layer (~0.8 GB) rebuilds. Takes ~30 seconds.

---

## Host Wrapper

### `~/bin/ros2`

Runs `ros2` commands inside the container from the host terminal:

```bash
docker exec isaac_ros_dev_container bash -c \
  "export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 $*"
```

---

## What's NOT in the Container

| Not included | Why |
|---|---|
| Unitree SDK / unitree_ros2 | Runs on the G1's Jetson Orin, not the workstation |
| System ROS 2 on host | Conflicts with Isaac Sim's bundled Python 3.11 rclpy |
| CycloneDDS config | Only needed for real robot mode (Orin handles this) |

---

## Troubleshooting

### Topics visible but no data flowing

FastDDS SHM issue. Check that the XML config exists and the env var is set:

```bash
docker exec isaac_ros_dev_container bash -c \
  "cat /etc/fastdds_no_shm.xml && echo '---' && echo \$FASTRTPS_DEFAULT_PROFILES_FILE"
```

If missing, the Dockerfile needs rebuilding — the config is baked into the image.

### Foxglove says "Check that WebSocket server is reachable"

foxglove-bridge isn't running. Check and restart:

```bash
docker exec isaac_ros_dev_container bash -c \
  "export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &"
```

### Container missing packages after restart

If you used `isaac-ros activate` to start the container, it recreated from the base image (losing customizations). Always use `docker compose up -d` instead.
