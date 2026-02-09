# Commands Reference

Quick reference for commonly used commands across the system.

---

## Docker

### Container Lifecycle

```bash
# Start container
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml up -d

# Stop container
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml down

# Attach a shell
docker exec -it -u admin isaac_ros_dev_container bash

# Quick alias (add to ~/.zshrc)
alias iros='docker exec -it -u admin isaac_ros_dev_container bash'
```

### Image Management

```bash
# Build custom image (base cached, ~30s)
cd ~/workspaces/isaac_ros-dev
docker build -t isaac-ros-dev:latest .

# List images
docker images

# Disk usage
docker system df

# Remove dangling images
docker image prune

# Remove all unused images
docker image prune -a
```

### Inspecting Containers

```bash
# List running containers
docker ps

# View container logs
docker logs isaac_ros_dev_container

# Run a command inside the container
docker exec isaac_ros_dev_container bash -c "command here"

# Check processes inside container
docker exec isaac_ros_dev_container ps aux
```

---

## ROS 2

All `ros2` commands run inside the container. Use the `~/bin/ros2` host wrapper or shell in first.

### Topics

```bash
# List all active topics
ros2 topic list

# Print live messages from a topic
ros2 topic echo /front_stereo_camera/left/camera_info

# Show publish rate
ros2 topic hz /front_stereo_camera/left/image_rect_color

# Show topic type and publisher/subscriber count
ros2 topic info /front_stereo_camera/left/image_rect_color
```

### Nodes

```bash
# List all running nodes
ros2 node list

# Show a node's publishers, subscribers, and services
ros2 node info /foxglove_bridge
```

### Launching Nodes

```bash
# AprilTag detection (GPU-accelerated)
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py

# AprilTag with Isaac Sim pipeline
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py

# Visual SLAM (cuVSLAM)
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# nvblox (3D reconstruction)
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py

# Foxglove Bridge (auto-starts, but manual restart if needed)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Services and Parameters

```bash
# List all services
ros2 service list

# List all parameters
ros2 param list

# Get a specific parameter
ros2 param get /foxglove_bridge port
```

---

## Isaac Sim

### Launching

```bash
# GUI mode (from desktop session)
conda activate isaaclab
isaacsim

# Headless mode (no display needed)
conda activate isaaclab
isaacsim isaacsim.exp.full.kit --/app/window/enabled=false --/app/livestream/enabled=false

# Headless with AprilTag warehouse scene
~/Documents/isaac-sim-scripts/headless-sample-scene.sh
```

### Useful Flags

| Flag | What it does |
|---|---|
| `--/app/window/enabled=false` | Disable GUI window |
| `--/app/livestream/enabled=false` | Disable WebRTC streaming |
| `isaacsim.exp.full.streaming.kit` | App config with streaming enabled |

---

## tmux

### Session Management

```bash
# Create a named session
tmux new-session -d -s session-name "command"

# List sessions
tmux ls

# Attach to a session
tmux attach -t session-name

# Detach from current session
Ctrl+B then D

# Kill a session
tmux kill-session -t session-name
```

### Pane Controls

| Shortcut | What it does |
|---|---|
| `Ctrl+B z` | Toggle zoom (fullscreen current pane) |
| `Ctrl+B x` | Kill current pane |
| `Ctrl+B d` | Detach from session |
| `Ctrl+B [` | Scroll mode (q to exit) |

---

## System

### Process Management

```bash
# Find Isaac Sim processes
ps aux | grep isaacsim

# Check what's using a port
lsof -i :8765

# Check GPU usage
nvidia-smi
```

### Networking

```bash
# Tailscale status
tailscale status

# Check DDS topics from host (wrapper)
~/bin/ros2 topic list

# Test Foxglove WebSocket
curl -s http://localhost:8765
```

### Zensical (Docs Site)

```bash
# Serve locally (from robot-docs directory)
cd ~/GIT/robot-docs
source .venv/bin/activate
zensical serve --dev-addr 0.0.0.0:8000

# Build static site
zensical build
```
