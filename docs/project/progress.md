# Current Progress

What's currently in progress and what's next.

## In Progress

### Sim-to-Real Deployment
Trained G1-29DOF velocity policy (7200 epochs) is ready. Blocked on:

1. Build `unitree_sdk2` C++ library on workstation
2. Compile `g1_ctrl` deploy binary (`deploy/robots/g1_29dof/`)
3. Point config at trained policy (`logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/`)
4. Connect robot via ethernet and run `./g1_ctrl --network <eth>`

See [Deploy Guide](../simulation/unitree/deploy.md).

## Next Steps

### Isaac Sim MCP Server
Control Isaac Sim from Claude Code via MCP. Plan exists, needs path updates for pip-installed Isaac Sim 5.1.

### Sim Performance to 1.0x Real-Time
Currently at 0.5x RTF with headless optimizations (640x480, RaytracedLighting, no reflections/AO, physics 4/1 iterations, Fabric enabled). May need further optimization or accept RTX 3090 hardware limit.

### Privacy Scanner
GitHub Actions workflow to scan commits for leaked credentials before they reach the public repo.

## Completed

- G1-29DOF locomotion policy trained with Isaac Lab — 7200 epochs, velocity policy exported to ONNX (Mar 2026)
- Nav2 autonomous navigation with MPPI speed tuning (Feb 2026)
- Nav2 auto-start on container boot via entrypoint script (Feb 2026)
- Static map->odom TF, odom TF bridge, cmd_vel relay auto-start (Feb 2026)
- Browser teleop via Foxglove joystick extension (fixed ROS2 schema build) (Feb 2026)
- Browser teleop via `teleop_twist_keyboard` in tmux (Feb 2026)
- Docs restructured from 7 tabs to 4 (Getting Started, Simulation, Infrastructure, Project)
- ROS 2 Topics and ROS Nodes documentation pages added
- Dockerfile updated with ROS 2 apt repo, teleop packages, entrypoints
- Isaac ROS container rebuilt with `teleop_twist_joy` and `teleop_twist_keyboard`
- Workstation OS upgrade from Ubuntu 22.04 to 24.04 (Feb 2026)
- Isaac Sim migration from standalone to pip install (Feb 2026)
- System ROS 2 removal (conflicted with Isaac Sim's Python 3.11)
- Isaac ROS container with AprilTag + Foxglove + H.264 NVENC
- Tailscale peer relay setup (tsrelay in Toronto)
- Architecture documentation for sim + real G1 workflows
- Isaac ROS Dockerfile created and container rebuilt (reproducible build, docker-compose managed)
