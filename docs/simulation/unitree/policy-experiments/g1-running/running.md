# G1 Running

## Purpose

Create a separate G1 running task without modifying Unitree's base walking task.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Running` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/running_env_cfg.py` |
| Base task | `Unitree-G1-29dof-Velocity` |
| Base config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_env_cfg.py` |
| Output experiment root | `logs/rsl_rl/unitree_g1_29dof_running/` |

`running_env_cfg.py` subclasses `RobotEnvCfg` from `velocity_env_cfg.py`. It inherits the robot asset, scene, observations, actions, terminations, terrain, contact sensors, sim timing, and most rewards from the base task, then overrides running-specific command and reward knobs.

## Tweaks From Base

| Area | Base velocity task | Running task |
|---|---|---|
| Initial forward command | `lin_vel_x=(-0.1, 0.1)` | `lin_vel_x=(0.2, 0.8)` |
| Initial lateral command | `lin_vel_y=(-0.1, 0.1)` | `lin_vel_y=(-0.05, 0.05)` |
| Initial yaw command | `ang_vel_z=(-0.1, 0.1)` | `ang_vel_z=(-0.1, 0.1)` |
| Forward curriculum limit | `lin_vel_x=(-0.5, 1.0)` | `lin_vel_x=(0.0, 2.0)` |
| Lateral curriculum limit | `lin_vel_y=(-0.3, 0.3)` | `lin_vel_y=(-0.2, 0.2)` |
| Yaw curriculum limit | `ang_vel_z=(-0.2, 0.2)` | `ang_vel_z=(-0.3, 0.3)` |
| Standing env fraction | `0.02` | `0.01` |
| Forward tracking weight | `1.0` | `1.5` |
| Yaw tracking weight | `0.5` | `0.25` |
| Gait weight | `0.5` | `0.75` |
| Gait period | `0.8` | `0.6` |
| Gait threshold | `0.55` | `0.5` |
| Foot clearance target | `0.1` | `0.13` |
| Action-rate penalty | `-0.05` | `-0.035` |
| Energy penalty | `-2e-5` | `-1.5e-5` |

The intent was to bias training toward straight forward running: higher forward velocity, less lateral/yaw complexity at the start, faster stepping cadence, more foot clearance, and slightly less punishment for energetic movement.

## Runs

Initial smoke test:

`logs/rsl_rl/unitree_g1_29dof_running/2026-05-08_09-58-58/`

First real pass:

`logs/rsl_rl/unitree_g1_29dof_running/2026-05-08_10-01-32_running_first_pass/`

That run was stopped around iteration 880. The latest durable checkpoint from that pass was:

`model_800.pt`

Training was resumed on May 14, 2026 with:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Running \
  --resume \
  --load_run 2026-05-08_10-01-32_running_first_pass \
  --run_name running_resume_may14 \
  --max_iterations 5000
```

This resumed run produced checkpoints through:

`logs/rsl_rl/unitree_g1_29dof_running/2026-05-14_03-05-14_running_resume_may14/model_1700.pt`

`model_1700.pt` was played back in the Isaac Sim GUI with 20 parallel robots:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-G1-29dof-Running \
  --num_envs 20 \
  --real-time \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_running/2026-05-14_03-05-14_running_resume_may14/model_1700.pt
```

After the GUI check, training was resumed again:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Running \
  --resume \
  --load_run 2026-05-14_03-05-14_running_resume_may14 \
  --run_name running_finish_may14 \
  --max_iterations 4100
```

The finish run writes to:

`logs/rsl_rl/unitree_g1_29dof_running/2026-05-14_03-36-49_running_finish_may14/`

That finish run was paused around iteration 2175. The latest durable checkpoint at that point was:

`model_2100.pt`

## Observed Behavior

During the first real pass, reward and episode length improved, but `Episode_Termination/bad_orientation` stayed high early in training. That means the policy was learning something, but many episodes still ended because the robot tipped or rotated too far.

The May 14 resumed run improved substantially. Before the GUI checkpoint test, TensorBoard showed rewards around the high 30s to low 40s, episode length near the 1000-step cap, and curriculum expansion up to about `1.9 m/s`. That made `model_1700.pt` a useful visual checkpoint. GUI playback looked strong enough to keep training.

After resuming the finish run, early metrics briefly restarted near the lower command curriculum range while the resumed run warmed back up. By around iteration 1760, reward was back around `41`, episode length was near `1000`, and `bad_orientation` was low.
