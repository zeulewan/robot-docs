# Isaac ROS Dockerfile

The Dockerfile and supporting files for the Isaac ROS container.

## Dockerfile

Located at `~/workspaces/isaac_ros-dev/Dockerfile`:

```dockerfile
FROM nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_<hash>-amd64

# Isaac ROS GPU perception + Foxglove H.264 + ffmpeg
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-isaac-ros-apriltag \
    ros-jazzy-isaac-ros-visual-slam \
    ros-jazzy-isaac-ros-nvblox \
    ros-jazzy-foxglove-compressed-video-transport \
    ros-jazzy-ffmpeg-encoder-decoder \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Create admin user
RUN useradd -m -s /bin/bash -G sudo admin \
    && echo "admin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# FastDDS no-SHM XML (fixes host<->container DDS transport)
COPY fastdds_no_shm.xml /etc/fastdds_no_shm.xml

# Persistent env var for FastDDS fix (login shells)
RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml' \
    > /etc/profile.d/fastdds-fix.sh \
    && chmod +x /etc/profile.d/fastdds-fix.sh

# Foxglove-bridge auto-start entrypoint
COPY 50-foxglove-bridge.sh /usr/local/bin/scripts/entrypoint_additions/
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/50-foxglove-bridge.sh

# H.264 NVENC republisher auto-start entrypoint
COPY 60-h264-republisher.sh /usr/local/bin/scripts/entrypoint_additions/
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/60-h264-republisher.sh

WORKDIR /workspaces/isaac_ros-dev
```

## docker-compose.yml

```yaml
services:
  isaac_ros:
    image: isaac-ros-dev:latest
    container_name: isaac_ros_dev_container
    runtime: nvidia
    network_mode: host
    ipc: host
    privileged: true
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - USERNAME=admin
    volumes:
      - /home/zeul/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev
    entrypoint: /usr/local/bin/scripts/workspace-entrypoint.sh
    command: bash -c "sleep infinity"
```

### Docker flags explained

| Flag | Why |
|---|---|
| `runtime: nvidia` | GPU access via NVIDIA Container Toolkit |
| `network_mode: host` | Share host network — DDS topics flow between Isaac Sim and container |
| `ipc: host` | Shared memory access (needed by some NVIDIA libraries) |
| `privileged: true` | Full device access (GPU, USB, etc.) |
| `entrypoint` | NVIDIA's startup script that runs entrypoint_additions scripts |
| `sleep infinity` | Keep container running in background |

## Build and Run

```bash
# Build (base image cached, only custom layer rebuilds ~30s)
cd ~/workspaces/isaac_ros-dev
docker build -t isaac-ros-dev:latest .

# Start
docker compose up -d

# Attach
docker exec -it -u admin isaac_ros_dev_container bash

# Stop
docker compose down
```

## Image Sizes

| Image | Size | What it is |
|---|---|---|
| `nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_...` | 38.8 GB | NVIDIA base (immutable) |
| `isaac-ros-dev:latest` | 39.6 GB | Base + custom layer (0.8 GB added) |

The base image is shared between both — Docker deduplicates layers. Total disk usage is ~39.6 GB, not 78.4 GB.
