# Demo Quick Reference

Step-by-step guide to launch the Carter warehouse simulation with Foxglove joystick teleop.

---

## Prerequisites

- Isaac Sim conda env activated: `conda activate isaaclab`
- Isaac ROS container image built (one-time)
- Foxglove Studio with the [foxglove-joystick extension](../assets/foxglove-joystick-ros2-fixed.foxe) installed (drag `.foxe` onto the Foxglove window)

---

## Step 1: Start Isaac Sim (Headless)

```bash
conda activate isaaclab
nohup bash ~/Documents/isaac-sim-scripts/headless-sample-scene.sh \
  > /tmp/isaac-sim-headless.log 2>&1 &
```

Wait for the scene to finish loading (~2 minutes). Check progress:

```bash
tail -f /tmp/isaac-sim-headless.log
```

Look for `Isaac Sim Headless is loaded` or physics-step messages to confirm it's ready.

To **stop** Isaac Sim, kill the python process by PID (not the bash wrapper):

```bash
# Find the PID
nvidia-smi --query-compute-apps=pid,name --format=csv,noheader | grep python3

# Kill it
kill <pid>
```

---

## Step 2: Start the Isaac ROS Container

```bash
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml up -d
```

This auto-starts all ROS services:

| Service | What it does |
|---|---|
| Foxglove bridge | WebSocket on port 8765 |
| H.264 republisher | GPU video encoding |
| Teleop twist joy | Foxglove joystick input |
| Odom TF bridge | Publishes `odom` → `base_link` transform |
| Static map → odom | Identity transform for navigation |
| Cmd vel relay | Relays `/cmd_vel` → `/cmd_vel_raw` |
| Nav2 | Full autonomous navigation stack |

Verify services are running:

```bash
ros2 node list
```

You should see nodes like `/foxglove_bridge`, `/bt_navigator`, `/controller_server`, etc.

---

## Step 3: Connect Foxglove

Open Foxglove Studio and connect to:

```
ws://localhost:8765
```

Or from a remote machine via Tailscale:

```
ws://100.101.214.44:8765
```

### Joystick Teleop

Add a **Joystick** panel — it publishes to `/joy`, and the `teleop_twist_joy` node (auto-started by the container) converts it to `/cmd_vel`.

| Joystick | Robot motion |
|---|---|
| Push up | Drive forward (1.0 m/s max) |
| Push down | Drive backward |
| Push left/right | Turn |

!!! warning "Drive carefully"
    The Carter robot gets physically stuck on collision in Isaac Sim — it can't reverse out. Start with low speeds and avoid walls.

### Other Useful Panels

- **3D** — visualize lidar (`/front_3d_lidar/lidar_points`), TF tree, camera feed
- **Image** — `/front_stereo_camera/left/compressed_video`
- **Publish** — send `/goal_pose` for autonomous navigation (schema: `geometry_msgs/msg/PoseStamped`)

---

## Shutting Down

Reverse order:

```bash
# 1. Stop the container (kills all ROS nodes)
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml down

# 2. Kill Isaac Sim (find the python process, not the bash wrapper)
nvidia-smi --query-compute-apps=pid,name --format=csv,noheader \
  | grep python3 | cut -d',' -f1 | xargs kill
```

---

## Managing ROS Nodes

### Check What's Running

```bash
# List all active nodes
ros2 node list

# Check a specific topic's publishers/subscribers
ros2 topic info /cmd_vel
```

### Kill Nodes

Kill processes inside the container — don't kill the `docker exec` wrapper from the host.

```bash
# Kill a specific node (e.g. Nav2 stack)
docker exec isaac_ros_dev_container bash -c \
  "pkill -9 -f 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager'"

# Kill a single node
docker exec isaac_ros_dev_container bash -c "pkill -9 -f foxglove_bridge"
```

!!! note
    After killing Nav2, wait ~30 seconds for DDS discovery to clear stale node registrations before relaunching.

### Start Nodes

Each command runs from the host — copy-paste directly.

```bash
# Foxglove bridge
docker exec isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &"
```

```bash
# Odom TF bridge
docker exec isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   python3 /workspaces/isaac_ros-dev/odom_tf_bridge.py &"
```

```bash
# Static map → odom transform
docker exec isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
   --ros-args -p use_sim_time:=true &"
```

```bash
# Cmd vel relay
docker exec isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   python3 /workspaces/isaac_ros-dev/cmd_vel_relay.py &"
```

```bash
# Teleop joystick
docker exec isaac_ros_dev_container bash -c \
  "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml && \
   ros2 run teleop_twist_joy teleop_node --ros-args \
   -p require_enable_button:=false -p axis_linear.x:=1 -p axis_angular.yaw:=0 \
   -p scale_linear.x:=-1.0 -p scale_angular.yaw:=1.0 &"
```

```bash
# Nav2 (autonomous navigation)
docker exec isaac_ros_dev_container bash /workspaces/isaac_ros-dev/launch_nav2.sh &
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| No topics visible | Isaac Sim not fully loaded yet — wait for physics step logs |
| Joystick not moving robot | Confirm `/joy` topic is being published: `ros2 topic echo /joy`. Check that the [ROS2-fixed extension](../assets/foxglove-joystick-ros2-fixed.foxe) is installed (not the original) |
| TF errors in Foxglove | Check `odom_tf_bridge` and `static_transform_publisher` are running: `ros2 node list` |
| Robot stuck after collision | Restart Isaac Sim — the Carter can't reverse out of collisions |
| Nav2 goals not working | Ensure you use a **Publish** panel (not 3D panel's Publish Pose) with schema `geometry_msgs/msg/PoseStamped` |
