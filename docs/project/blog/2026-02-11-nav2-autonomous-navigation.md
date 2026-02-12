# Nav2 Autonomous Navigation and MPPI Tuning

*2026-02-11*

Got the Carter robot driving itself around the warehouse. Nav2 is now fully integrated -- path planning, obstacle avoidance, and goal tracking all working through Foxglove. Most of the work was getting MPPI (the velocity controller) to actually go faster than a crawl.

## Setting Up Nav2

The Isaac ROS container already had all 36 Nav2 packages installed (v1.3.9). The main challenge was wiring everything together:

- **Costmaps** use VoxelLayer fed by the 3D lidar (`/front_3d_lidar/lidar_points`). No static map needed -- obstacles build up from live sensor data as the robot moves.
- **Localization** uses a static identity `map -> odom` transform. cuVSLAM can provide this instead, but it drifts over long runs. The static transform is simpler and reliable for local navigation.
- **TF chain** required two helper scripts: `odom_tf_bridge.py` (converts `/chassis/odom` to a TF broadcast) and a `static_transform_publisher` for `map -> odom`. Both must use `use_sim_time:=true` or the timestamps don't match.

Nav2 1.3.9's stock `navigation_launch.py` requires `nav2_route` which isn't in the NVIDIA apt repos, so I wrote a custom launch file that skips `route_server`, `collision_monitor`, and `docking_server`.

## MPPI Speed Tuning

This is where most of the time went. Out of the box, MPPI crawls at ~0.05 m/s despite being configured for 2.0 m/s max. After about 10 iterations of parameter changes and research into the MPPI source code, here's what I found:

### temperature was the #1 bottleneck

MPPI works by sampling thousands of random trajectories, scoring them with "critics" (penalty functions), and picking the best via a temperature-weighted average. The `temperature` parameter controls how aggressively it favors low-cost trajectories.

At `temperature: 0.1`, the optimizer gets trapped. Once it converges on ~0.4 m/s, any faster trajectory incurs slightly higher per-step critic costs and gets exponentially suppressed. It's stuck in a local minimum. Changing to `temperature: 0.3` (the default) let it properly explore faster solutions.

### Critics accumulate differently

Some critics evaluate every step of the trajectory (56 penalties summed), others only check the endpoint. Per-step critics inherently penalize speed -- a faster trajectory traverses more costmap cells, accumulating more cost even on a clear path. The key insight: `PathFollowCritic` is endpoint-only and is THE main forward-driving force. Setting its weight to 50.0 makes it dominate the per-step accumulators.

### vx_std has a sweet spot

`vx_std` controls the random noise added to velocity samples. Too narrow (0.2) and the optimizer takes forever to explore higher speeds. Too wide (0.8) and samples cancel out -- some go fast-forward, others fast-backward, and the weighted average nets to near-zero. The sweet spot for `vx_max=2.0` is around **0.5**.

### Costmap/prune/horizon must be consistent

There's a hard math relationship: `prune_distance` and the local costmap must both exceed `vx_max * time_steps * model_dt` (5.6m at our settings). If the controller can only see 3.5m of path, it physically can't plan fast trajectories. This was the original speed limiter before I found the temperature issue.

### Compute budget matters at 0.5x RTF

Isaac Sim runs at 0.5x real-time on the RTX 3090. Setting `batch_size=3000` with `iteration_count=2` dropped the control rate to 12 Hz and made the robot erratic. `batch_size=2000` with `iteration_count=1` sustains ~15-20 Hz and drives smoothly.

## Auto-Start on Container Boot

Everything now starts automatically when the container launches, via numbered scripts in `/usr/local/bin/scripts/entrypoint_additions/`:

| Script | Process |
|---|---|
| `70-teleop-twist-joy.sh` | Foxglove joystick teleop |
| `75-odom-tf-bridge.sh` | odom -> base_link TF bridge |
| `76-static-map-odom.sh` | Static map -> odom identity TF |
| `77-cmd-vel-relay.sh` | cmd_vel relay (pass-through for OmniGraph remap) |
| `80-nav2.sh` | Full Nav2 stack |

Teleop and Nav2 coexist on `/cmd_vel` -- the joystick only publishes when touched, so Nav2 commands pass through when the stick is idle.

## Sending Goals from Foxglove

The 3D panel's built-in "Publish Pose" doesn't work (it uses `foxglove.PoseInFrame`, not ROS). Instead, add a **Publish panel** with topic `/goal_pose` and schema `geometry_msgs/msg/PoseStamped`. Set a position in the map frame and the robot plans a path and drives there.

## Current Results

The robot navigates to goals at **~0.4 m/s peak**, **~0.3 m/s odom**, with cruise around 0.15-0.20 m/s. The main remaining bottleneck is a known Nav2 issue ([#3303](https://github.com/ros-navigation/navigation2/issues/3303)) -- the controller uses wall-clock time for its loop rate, but the sim runs at 0.5x real-time, causing prediction horizon mismatches.

## What's Next

- Investigate whether the WallRate mismatch can be worked around at the sim level (faster RTF)
- Try frontier exploration for autonomous mapping
- Isaac Sim MCP server integration
