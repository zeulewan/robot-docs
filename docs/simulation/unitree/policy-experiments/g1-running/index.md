# G1 Running Policy Family

This experiment family extends Unitree's base G1 29DOF velocity task into progressively faster forward locomotion tasks without modifying the upstream walking task.

| Variant | Task ID | Config | Notes |
|---|---|---|---|
| [Running](running.md) | `Unitree-G1-29dof-Running` | `running_env_cfg.py` | First running task. Changes command range, gait cadence, clearance, and running reward weights. |
| [Fast running](fast-running.md) | `Unitree-G1-29dof-Running-Fast` | `fast_running_env_cfg.py` | Warm-started from running; increases forward curriculum to `3.0 m/s`. |
| [Sprint 10 m/s](sprint-10ms.md) | `Unitree-G1-29dof-Sprint-10ms` | `sprint_10ms_env_cfg.py` | Paused at `model_20500.pt`; reached about `7.2 m/s` curriculum, but gait still needs tuning. |
| [Sprint 10 m/s gait cleanup](sprint-10ms.md#gait-cleanup-variant) | `Unitree-G1-29dof-Sprint-10ms-Gait` | `sprint_10ms_env_cfg.py` | New variant that relaxes waist/hip penalties and gates velocity curriculum on fall/reset stability. |

## Shared Setup

| Item | Value |
|---|---|
| Base task | `Unitree-G1-29dof-Velocity` |
| Base config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_env_cfg.py` |
| Trainer | RSL-RL PPO |
| Robot | Plain G1 29DOF |
| Observation/action interface | Same as the base velocity task |
| Policy observation shape | `480` |
| Policy action shape | `29` joint action targets |
| Full actor-critic checkpoint parameters | `832,571` |

The deployed actor portion of the policy is smaller than the full training checkpoint. The actor network has `414,237` parameters and maps the 480-value policy observation to 29 joint action targets. The critic has `418,305` parameters and is used during PPO training only. The remaining `29` parameters are the learned action standard deviation used for exploration.

## Lineage

```text
Unitree-G1-29dof-Velocity
  -> Unitree-G1-29dof-Running
     -> Unitree-G1-29dof-Running-Fast
        -> Unitree-G1-29dof-Sprint-10ms
           -> Unitree-G1-29dof-Sprint-10ms-Gait
```

Each faster variant is a separate task ID and writes to a separate experiment folder under `logs/rsl_rl/`.

## Key Metrics

Watch these in TensorBoard:

| Metric | Meaning |
|---|---|
| `Train/mean_reward` | Overall learning signal; should trend upward |
| `Train/mean_episode_length` | How long robots survive; higher is better |
| `Metrics/base_velocity/error_vel_xy` | Forward/lateral velocity tracking error; lower is better |
| `Curriculum/lin_vel_cmd_levels` | How far the velocity curriculum has expanded |
| `Curriculum/lin_vel_cmd_stability/failure_rate` | For stability-gated tasks, recent fall-like reset rate |
| `Episode_Termination/bad_orientation` | Falling/tipping termination; should drop over time |

Use playback before trusting a checkpoint. Good TensorBoard curves can still hide ugly gaits, skating, terrain-edge artifacts, or reward hacking.
