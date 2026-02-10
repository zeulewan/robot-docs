# Current Progress

What's currently in progress and what's next.

## In Progress

### Browser Teleop via Foxglove Joystick
Drive the Carter robot from a browser using the foxglove-joystick extension. The original extension has a ROS2 schema bug (`sensor_msgs/Joy` vs `sensor_msgs/msg/Joy`). Built a fixed `.foxe` from [MSSergeev's fork](https://github.com/MSSergeev/foxglove-joystick) -- needs to be installed and tested.

**What's done:**

- `teleop_twist_joy` installed in container with auto-start entrypoint
- Joy schema seed burst at container boot
- Fixed extension built from source (`~/Desktop/foxglove-joystick-ros2-fixed.foxe`)

**What's left:**

- Install the fixed `.foxe` in Foxglove (drag and drop)
- Test joystick → `/joy` → `teleop_twist_joy` → `/cmd_vel` → robot
- If it works, remove the seed burst from the entrypoint (shouldn't be needed with correct schema)

## Next Steps

### Isaac Sim MCP Server
Control Isaac Sim from Claude Code via MCP. Plan exists, needs path updates for pip-installed Isaac Sim 5.1.

### Performance Tuning to 60fps
- Lower viewport resolution (1280x720 -> 960x540)
- Reduce physics solver iterations (4 -> 2)
- Disable unnecessary rendering (reflections, translucency, AO)
- Set physics CPU threads to 1

### Privacy Scanner
GitHub Actions workflow to scan commits for leaked credentials before they reach the public repo.

## Completed

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
