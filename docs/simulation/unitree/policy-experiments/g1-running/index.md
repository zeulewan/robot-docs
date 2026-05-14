# G1 Running Policy

## Purpose

This experiment creates a separate G1 running task without modifying Unitree's base walking task.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Running` |
| Fast task ID | `Unitree-G1-29dof-Running-Fast` |
| Base task | `Unitree-G1-29dof-Velocity` |
| Base config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_env_cfg.py` |
| Running config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/running_env_cfg.py` |
| Fast running config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/fast_running_env_cfg.py` |
| Trainer | RSL-RL PPO |
| Observation/action interface | Same as the base velocity task |
| Policy parameters | `832,571` total trainable parameters in the RSL-RL actor-critic checkpoint |

`running_env_cfg.py` subclasses `RobotEnvCfg` from `velocity_env_cfg.py`. That means the running task inherits the robot asset, scene, observations, actions, terminations, terrain, contact sensors, sim timing, and most rewards from the base task, then overrides only the running-specific knobs.

The deployed actor portion of the policy is smaller than the full training checkpoint. The actor network has `414,237` parameters and maps the 480-value policy observation to 29 joint action targets. The critic has `418,305` parameters and is used during PPO training only. The remaining `29` parameters are the learned action standard deviation used for exploration.

## Tweaks From Base

The first running pass changed several things at once:

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

## Fast Running Variant

The fast-running variant was added after the normal running policy looked good in GUI playback. It does not replace `Unitree-G1-29dof-Running`; it is a separate task:

`Unitree-G1-29dof-Running-Fast`

`fast_running_env_cfg.py` subclasses `RunningRobotEnvCfg`, then changes only the command ranges:

| Area | Running task | Fast running task |
|---|---|---|
| Initial forward command | `lin_vel_x=(0.2, 0.8)` | `lin_vel_x=(0.8, 1.6)` |
| Initial lateral command | `lin_vel_y=(-0.05, 0.05)` | `lin_vel_y=(-0.05, 0.05)` |
| Initial yaw command | `ang_vel_z=(-0.1, 0.1)` | `ang_vel_z=(-0.1, 0.1)` |
| Forward curriculum limit | `lin_vel_x=(0.0, 2.0)` | `lin_vel_x=(0.0, 3.0)` |
| Lateral curriculum limit | `lin_vel_y=(-0.2, 0.2)` | `lin_vel_y=(-0.15, 0.15)` |
| Yaw curriculum limit | `ang_vel_z=(-0.3, 0.3)` | `ang_vel_z=(-0.25, 0.25)` |

The fast task intentionally keeps the existing gait, clearance, energy, and action-rate reward settings from the normal running task. The first question is whether the learned gait can stretch to higher forward speeds before adding more reward changes.

## Current Training Runs

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

This resumed run produced checkpoints through `model_1700.pt`. `model_1700.pt` was played back in the Isaac Sim GUI with 20 parallel robots:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-G1-29dof-Running \
  --num_envs 20 \
  --real-time \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_running/2026-05-14_03-05-14_running_resume_may14/model_1700.pt
```

Visual playback looked strong: the robots were upright and moving cleanly enough to continue training rather than rolling back the reward changes.

After the GUI check, training was resumed again from the May 14 run to finish the remaining iterations:

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

The fast-running task was warm-started from that policy by copying the checkpoint into the fast experiment root:

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

## Observed Behavior

During the first real pass, reward and episode length improved, but `Episode_Termination/bad_orientation` stayed high early in training. That means the policy was learning something, but many episodes still ended because the robot tipped or rotated too far.

The May 14 resumed run improved substantially. Before the GUI checkpoint test, TensorBoard showed rewards around the high 30s to low 40s, episode length near the 1000-step cap, and curriculum expansion up to about `1.9 m/s`. That made `model_1700.pt` a useful visual checkpoint. The GUI playback looked good enough to keep training.

After resuming the finish run, early metrics briefly restarted near the lower command curriculum range while the resumed run warmed back up. By around iteration 1760, reward was back around `41`, episode length was near `1000`, and `bad_orientation` was low.

This first pass changed many variables at once, so it is not a controlled ablation. If the resumed run does not stabilize, the next pass should simplify the experiment:

- start with a smaller command increase from the base velocity task
- keep gait and clearance closer to the base task
- train one change at a time
- compare TensorBoard curves and playback checkpoints against the base walking task

## Key Metrics

Watch these in TensorBoard:

| Metric | Meaning |
|---|---|
| `Train/mean_reward` | Overall learning signal; should trend upward |
| `Train/mean_episode_length` | How long robots survive; higher is better |
| `Metrics/base_velocity/error_vel_xy` | Forward/lateral velocity tracking error; lower is better |
| `Curriculum/lin_vel_cmd_levels` | How far the velocity curriculum has expanded |
| `Episode_Termination/bad_orientation` | Falling/tipping termination; should drop over time |

Use playback before trusting a checkpoint. Good TensorBoard curves can still hide ugly gaits, skating, or reward hacking.
