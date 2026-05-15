# G1 Fast Running

## Purpose

Push the working running policy to a higher forward velocity curriculum while keeping the running reward settings mostly unchanged.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Running-Fast` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/fast_running_env_cfg.py` |
| Parent task | `Unitree-G1-29dof-Running` |
| Output experiment root | `logs/rsl_rl/unitree_g1_29dof_running_fast/` |
| Warm-start checkpoint | `logs/rsl_rl/unitree_g1_29dof_running/2026-05-14_03-36-49_running_finish_may14/model_2100.pt` |

`fast_running_env_cfg.py` subclasses `RunningRobotEnvCfg`, then changes only the command ranges. It intentionally keeps the gait, clearance, energy, action-rate, and termination settings from the normal running task.

## Tweaks From Running

| Area | Running task | Fast running task |
|---|---|---|
| Initial forward command | `lin_vel_x=(0.2, 0.8)` | `lin_vel_x=(0.8, 1.6)` |
| Initial lateral command | `lin_vel_y=(-0.05, 0.05)` | `lin_vel_y=(-0.05, 0.05)` |
| Initial yaw command | `ang_vel_z=(-0.1, 0.1)` | `ang_vel_z=(-0.1, 0.1)` |
| Forward curriculum limit | `lin_vel_x=(0.0, 2.0)` | `lin_vel_x=(0.0, 3.0)` |
| Lateral curriculum limit | `lin_vel_y=(-0.2, 0.2)` | `lin_vel_y=(-0.15, 0.15)` |
| Yaw curriculum limit | `ang_vel_z=(-0.3, 0.3)` | `ang_vel_z=(-0.25, 0.25)` |

The first question was whether the learned running gait could stretch to higher forward speeds before adding more reward changes.

## Runs

The fast-running task was warm-started by copying the normal running checkpoint into the fast experiment root:

`logs/rsl_rl/unitree_g1_29dof_running_fast/from_running_model_2100/model_2100.pt`

Fast training was started with:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Running-Fast \
  --resume \
  --load_run from_running_model_2100 \
  --checkpoint model_2100.pt \
  --run_name fast_from_running_2100 \
  --max_iterations 3000
```

The fast run writes to:

`logs/rsl_rl/unitree_g1_29dof_running_fast/2026-05-14_03-57-12_fast_from_running_2100/`

The completed fast model reached:

`logs/rsl_rl/unitree_g1_29dof_running_fast/2026-05-14_03-57-12_fast_from_running_2100/model_5099.pt`

Playback exported:

```text
logs/rsl_rl/unitree_g1_29dof_running_fast/2026-05-14_03-57-12_fast_from_running_2100/exported/policy.pt
logs/rsl_rl/unitree_g1_29dof_running_fast/2026-05-14_03-57-12_fast_from_running_2100/exported/policy.onnx
```

## Observed Behavior

The completed fast run reached the full `3.0 m/s` curriculum. Final TensorBoard values around `model_5099.pt` were:

| Metric | Approximate final value |
|---|---|
| `Train/mean_reward` | `29.6` |
| `Train/mean_episode_length` | `986 / 1000` |
| `Metrics/base_velocity/error_vel_xy` | `0.49` |
| `Metrics/base_velocity/error_vel_yaw` | `2.33` |
| `Curriculum/lin_vel_cmd_levels` | `3.0 m/s` |
| `Episode_Termination/bad_orientation` | about `2%` |
| `Episode_Termination/time_out` | about `93%` |

GUI playback was visually interesting and stable enough to justify a 10 m/s stress-test variant.

The first playback used the inherited small play terrain, which can let robots run off the edge at high speed. This is a playback artifact risk: training used a larger terrain than the default play config. The later sprint task explicitly enlarges the play terrain.
