# Current Progress

What's currently in progress and what's next.

## In Progress

(Nothing actively in progress.)

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

- Workstation OS upgrade from Ubuntu 22.04 to 24.04 (Feb 2026)
- Isaac Sim migration from standalone to pip install (Feb 2026)
- System ROS 2 removal (conflicted with Isaac Sim's Python 3.11)
- Isaac ROS container with AprilTag + Foxglove + H.264 NVENC
- Tailscale peer relay setup (tsrelay in Toronto)
- Architecture documentation for sim + real G1 workflows
- Full NVIDIA Docker image pulled for container migration
- Isaac ROS Dockerfile created and container rebuilt (reproducible build, docker-compose managed)
