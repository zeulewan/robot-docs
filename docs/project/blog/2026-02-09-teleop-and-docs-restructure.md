# Teleop Control and Docs Restructure

*2026-02-09*

Two things today: getting browser-based robot control working through Foxglove, and restructuring the docs site from 7 tabs down to 4.

## Teleop Control

The goal is to drive the Carter robot from a browser using Foxglove Studio. There are two paths to this:

### teleop_twist_keyboard (working)

Added `ros-jazzy-teleop-twist-keyboard` to the Dockerfile. Run it in a tmux session inside the container and it publishes `geometry_msgs/Twist` directly to `/cmd_vel`. WASD-style control, sends zero velocity on key release so the robot actually stops. This works reliably.

### Foxglove Joystick Extension (in progress)

Installed the [foxglove-joystick](https://github.com/joshnewans/foxglove-joystick) extension for browser-based joystick control. This publishes `sensor_msgs/Joy` messages, which need converting to `Twist` via `teleop_twist_joy`.

Added to the container:

- `ros-jazzy-teleop-twist-joy` (required adding the standard ROS 2 apt repo to the Dockerfile -- NVIDIA's base image only includes their own repos)
- `70-teleop-twist-joy.sh` entrypoint that starts the Joy-to-Twist converter with deadman switch disabled
- A seed burst of Joy messages at startup to register the schema with the foxglove bridge

**The Dockerfile also needed a fix**: NVIDIA's base image ships with a broken yarn apt repo (expired GPG key) that causes `apt-get update` to fail. Added `rm -f /etc/apt/sources.list.d/yarn.list` before the update.

### The ROS1 vs ROS2 Schema Bug

The joystick extension kept failing with "Unknown message definition for /joy". After debugging through multiple approaches (persistent publishers, burst seeding, checking GitHub issues), found the root cause in [PR #13](https://github.com/joshnewans/foxglove-joystick/pull/13):

The original extension advertises with `sensor_msgs/Joy` (ROS1 schema name). ROS2 requires `sensor_msgs/msg/Joy`. The foxglove bridge rejects the advertise because it doesn't recognize the schema.

Built the fixed extension from [MSSergeev's fork](https://github.com/MSSergeev/foxglove-joystick) which adds ROS2 auto-detection, 20 Hz configurable publish rate, and axis deadzone filtering. The `.foxe` file is ready to install -- will test next session.

## Docs Restructure

Reorganized the site from 7-8 nav tabs to 4:

| Before | After |
|---|---|
| Getting Started, Isaac Sim, Isaac ROS, Workstation, Networking, Project, Blog, commands.md | Getting Started, Simulation, Infrastructure, Project |

Related sections are now nested as subfolders with bold section headings in the sidebar:

- **Simulation** contains Isaac Sim and Isaac ROS
- **Infrastructure** contains Workstation, Networking, and Commands
- **Project** contains Progress, About, and Blog

Created landing pages for Simulation and Infrastructure with grid cards linking to subsections.

## New Documentation Pages

- **[ROS 2 Topics](../../simulation/isaac-sim/topics.md)** -- all topics published by the Carter Warehouse scene (sensors, robot state, control, TF, system)
- **[ROS Nodes](../../simulation/isaac-ros/nodes.md)** -- all nodes in the Isaac ROS container (auto-start and on-demand) with data flow diagram
- Updated the **[Dockerfile](../../simulation/isaac-ros/dockerfile.md)** page to match current state (ROS 2 repo, teleop packages, entrypoints)
