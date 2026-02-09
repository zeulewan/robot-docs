# Getting Started

Start here if you're new to this system. Covers what everything is, how it fits together, and where to go next.

Robotics simulation and perception stack for the **Unitree G1 humanoid robot**. Start here if ROS, Isaac Sim, or Docker are new to you.

## What this system does

Develop software for a bipedal humanoid robot (Unitree G1) that walks, sees, and interacts with its environment. Two modes:

| Mode | What's running | Where |
|---|---|---|
| **Simulation** | A virtual G1 in a physics simulator | Workstation (RTX 3090) |
| **Real robot** | Physical G1 with cameras, IMU, motors | G1's onboard Jetson Orin (runs autonomously) |

In simulation mode, the workstation runs both the simulator and perception. In real robot mode, the G1 runs everything onboard on its Jetson Orin -- the workstation is only used for simulation and training.

## The key software

### ROS 2 (Robot Operating System 2)

Middleware framework for robotics. Programs ("nodes") communicate by publishing and subscribing to named data channels ("topics"). A camera node publishes image data on `/camera/image`, a SLAM node subscribes to it.

- **Not an OS** -- a set of libraries and tools that run on Linux
- **DDS** -- transport layer. Nodes discover each other automatically over the network, no central server
- **Version:** Jazzy (latest LTS)

### Isaac Sim

NVIDIA's robot simulator. Renders a 3D world with accurate physics (contacts, gravity, friction) and simulates sensors (cameras, IMU, lidar). The simulated G1 publishes the same ROS 2 topics as the real robot -- downstream code doesn't know the difference.

- Runs on workstation GPU (RTX 3090)
- GUI mode needs a desktop session; headless mode available for training
- Built-in ROS 2 bridge publishes sensor data

### Isaac ROS

GPU-accelerated ROS 2 packages from NVIDIA for robot perception:

| Package | What it does |
|---|---|
| **cuVSLAM** | Visual SLAM -- builds a map and tracks the robot's position using cameras |
| **AprilTag** | Detects fiducial markers in camera images |
| **nvblox** | Builds a 3D occupancy map from depth cameras |

Run inside a Docker container on the workstation in simulation mode. For the real G1, these same packages run onboard the Jetson Orin.

### Docker

All Isaac ROS packages run in a Docker container -- an isolated Linux environment with its own libraries. Avoids version conflicts with the host system. Full GPU access via `nvidia-container-toolkit`.

### Foxglove

Browser-based visualization tool. Connects to a WebSocket bridge (port 8765) in the Isaac ROS container to display camera feeds, 3D maps, robot state, and sensor data in real time. No install needed -- just open a browser.

## How it all connects

```mermaid
flowchart LR
    subgraph WS["Workstation"]
        SIM["Isaac Sim<br/>Simulated G1 robot<br/>cameras, IMU, joints<br/>physics + rendering<br/>GPU"]
        ROS["Isaac ROS Container<br/>SLAM, AprilTag<br/>nvblox, nav2<br/>Foxglove bridge :8765"]
    end
    FOX["Foxglove Studio<br/>(any browser)"]

    SIM -- "DDS" --> ROS
    ROS -- "DDS" --> SIM
    ROS -- "WebSocket :8765" --> FOX
```

In simulation mode, Isaac Sim replaces the real robot. In real robot mode, the G1's Jetson Orin publishes the same ROS 2 topics over Ethernet instead.

## Accessing the workstation

| Method | Use case | Details |
|---|---|---|
| **Moonlight** | Full desktop (best for Isaac Sim GUI) | Connect via Sunshine streaming server |
| **SSH** | Terminal only | `ssh workstation` (via Tailscale) |
| **Foxglove** | View sensor data from any browser | `ws://workstation:8765` |

All remote access works over [Tailscale](../guides/tailscale-setup.md) (mesh VPN) -- no need to be on the same network.

## Where to go next

1. **[Architecture](architecture.md)** -- detailed data flow diagrams for simulation and real robot modes
2. **[Current Setup](../reference/current-setup.md)** -- exact software versions and configuration
3. **[Isaac Sim](../guides/isaac-sim-setup.md)** -- launching the simulator, ROS 2 bridge, troubleshooting
4. **[Isaac ROS](../guides/isaac-ros-container.md)** -- the Docker container, what's inside, how to use it
5. **[Current Progress](../project/progress.md)** -- what's being worked on right now
