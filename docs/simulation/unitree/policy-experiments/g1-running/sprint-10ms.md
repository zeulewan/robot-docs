# G1 Sprint 10 m/s

## Purpose

Stress-test the learned fast-running gait with a forward velocity curriculum up to `10.0 m/s`.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Sprint-10ms` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/sprint_10ms_env_cfg.py` |
| Parent task | `Unitree-G1-29dof-Running-Fast` |
| Output experiment root | `logs/rsl_rl/unitree_g1_29dof_sprint_10ms/` |
| Warm-start checkpoint | `logs/rsl_rl/unitree_g1_29dof_running_fast/2026-05-14_03-57-12_fast_from_running_2100/model_5099.pt` |

This is a simulation stress test, not a deployable target. `10 m/s` over a 20 second episode can cover about 200 m, so terrain size matters. The sprint task uses larger training and playback terrain to reduce edge-of-terrain failures.

## Tweaks From Fast Running

| Area | Fast running task | Sprint 10 m/s task |
|---|---|---|
| Initial forward command | `lin_vel_x=(0.8, 1.6)` | `lin_vel_x=(2.0, 4.0)` |
| Initial lateral command | `lin_vel_y=(-0.05, 0.05)` | `lin_vel_y=(-0.05, 0.05)` |
| Initial yaw command | `ang_vel_z=(-0.1, 0.1)` | `ang_vel_z=(-0.05, 0.05)` |
| Forward curriculum limit | `lin_vel_x=(0.0, 3.0)` | `lin_vel_x=(0.0, 10.0)` |
| Lateral curriculum limit | `lin_vel_y=(-0.15, 0.15)` | `lin_vel_y=(-0.10, 0.10)` |
| Yaw curriculum limit | `ang_vel_z=(-0.25, 0.25)` | `ang_vel_z=(-0.15, 0.15)` |
| Training terrain grid | `9 x 21`, `8 m` tile size | `21 x 21`, `20 m` tile size |
| Playback terrain grid | `2 x 10`, `8 m` tile size | `21 x 21`, `20 m` tile size |
| Forward tracking weight | `1.5` | `2.0` |
| Forward tracking std | `0.6` | `1.0` |
| Yaw tracking weight | `0.25` | `0.10` |
| Gait weight | `0.75` | `0.50` |
| Gait period | `0.6` | `0.45` |
| Gait threshold | `0.5` | `0.45` |
| Foot clearance target | `0.13` | `0.16` |
| Feet slide penalty | `-0.2` | `-0.1` |
| Action-rate penalty | `-0.035` | `-0.02` |
| Energy penalty | `-1.5e-5` | `-1.0e-5` |
| Flat orientation penalty | `-5.0` | `-3.0` |
| Bad-orientation termination angle | `0.8 rad` | `1.0 rad` |

The sprint task tightens lateral and yaw commands so the first experiment asks a simpler question: can the existing fast-running gait be extended into mostly straight high-speed locomotion?

## First Sprint Run

Checkpoint staging:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/from_fast_model_5099/model_5099.pt`

Initial sprint command:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Sprint-10ms \
  --resume \
  --load_run from_fast_model_5099 \
  --checkpoint model_5099.pt \
  --run_name sprint10_from_fast_5099 \
  --max_iterations 3000
```

Because RSL-RL counts from the loaded checkpoint iteration when resuming, `model_5099.pt` plus `--max_iterations 3000` targets about iteration `8099`.

The first sprint run wrote to:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/2026-05-14_12-01-02_sprint10_from_fast_5099/`

Launch log:

`logs/rsl_rl/unitree_g1_sprint10_train_20260514_120056.log`

That first run was stopped after `model_5200.pt` was saved so the longer run could be restarted with a 10k total-iteration target and the larger terrain config.

## 10k Big-Terrain Run

Checkpoint staging:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/from_sprint_model_5200_big_terrain/model_5200.pt`

Restart command:

```bash
TERM=xterm /home/zeul/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Sprint-10ms \
  --resume \
  --load_run from_sprint_model_5200_big_terrain \
  --checkpoint model_5200.pt \
  --run_name sprint10_to_10k_big_terrain \
  --max_iterations 4800
```

`model_5200.pt` plus `--max_iterations 4800` targets iteration `10000`.

The 10k target run writes to:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/2026-05-14_12-05-24_sprint10_to_10k_big_terrain/`

Launch log:

`logs/rsl_rl/unitree_g1_sprint10_train_10k_big_terrain_20260514_120517.log`

Saved `env.yaml` terrain confirmation:

```yaml
size: [20.0, 20.0]
num_rows: 21
num_cols: 21
```

## Curriculum-To-Done Continuation

The 10k run did not complete the `10.0 m/s` curriculum, so a continuation task was registered as:

`Unitree-G1-29dof-Sprint-10ms-Curriculum-Resume`

The continuation resumed from:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/2026-05-14_14-19-22_sprint10_curriculum_to_done_from_8000/model_10200.pt`

The new run wrote to:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/2026-05-14_15-46-40_sprint10_curriculum_to_done_from_10200/`

Launch log:

`logs/rsl_rl/unitree_g1_sprint10_curriculum_to_done_from_10200_20260514_154634.log`

The watcher was configured to stop only after `Curriculum/lin_vel_cmd_levels` reaches `10.0` and a checkpoint at or after that point exists.

## Pause At Model 20500

The continuation was manually paused on May 14, 2026 after the live log reached about iteration `20555/58000`. The latest durable checkpoint at pause time was:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms/2026-05-14_15-46-40_sprint10_curriculum_to_done_from_10200/model_20500.pt`

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../../assets/g1-sprint-model-20500.mp4" type="video/mp4">
</video>

Playback from `model_20500.pt`. This clip is a qualitative checkpoint review, not a fixed-speed measurement.

Observed status at pause:

| Metric | Value |
|---|---|
| Latest live iteration | `20555/58000` |
| Latest checkpoint | `model_20500.pt` |
| Velocity curriculum | about `7.2 m/s` |
| Terrain curriculum | about `10.4` |
| Mean episode length | roughly `500-600` steps |
| `bad_orientation` termination | still high, about `0.58` |

Playback looks usable enough to confirm the pipeline and high-speed curriculum are working, but the gait is not yet polished. The robot moves forward and stays upright in some rollouts, but the motion still needs reward/config tuning to look like a cleaner running gait. Likely next tuning areas are gait cadence, feet clearance, base-height/orientation penalties, action-rate/energy regularization, and command sampling during playback so videos report an exact commanded speed.

Important caveat: the playback config samples commands from the play task ranges, so a video from this checkpoint should be described as a policy playback from a model trained to about the `7.2 m/s` curriculum point, not as a verified fixed-speed `7.2 m/s` or `10.0 m/s` run.

## Gait Cleanup Variant

Task ID:

`Unitree-G1-29dof-Sprint-10ms-Gait`

Config:

`source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/sprint_10ms_env_cfg.py`

Reason for the variant: the first sprint policy can move quickly, but the gait is stiff and unstable. The waist is not locked out of the action space, but the original reward setup strongly discourages waist motion and hip roll/yaw deviation. Combined with flat-orientation, base-height, action-rate, and energy penalties, the policy is biased toward a conservative upright shuffle instead of a cleaner running gait.

Changes from `Unitree-G1-29dof-Sprint-10ms`:

| Area | Sprint 10 m/s | Gait cleanup |
|---|---:|---:|
| Forward command lower limit | `0.0 m/s` | `2.0 m/s` |
| Forward tracking weight | `2.0` | `2.25` |
| Forward tracking std | `1.0` | `0.7` |
| Waist deviation penalty | `-1.0` | `-0.20` |
| Hip roll/yaw deviation penalty | `-1.0` | `-0.35` |
| Gait weight | `0.50` | `0.35` |
| Gait period | `0.45 s` | `0.38 s` |
| Foot clearance target | `0.16 m` | `0.17 m` |
| Action-rate penalty | `-0.02` | `-0.015` |
| Energy penalty | `-1.0e-5` | `-8.0e-6` |
| Flat-orientation penalty | `-3.0` | `-2.0` |
| Base-height penalty | `-10.0` | `-6.0` |
| Base-height target | `0.78 m` | `0.74 m` |

The forward curriculum is now stability-gated with `stable_lin_vel_cmd_levels`. It only expands the maximum forward command when the tracking reward is high enough and the recent fall-like reset rate is low enough. The main stability signal is the fraction of environments whose latest episode ended from `bad_orientation` or `base_height`, with a default gate of `max_failure_rate=0.20`. The curriculum also logs `Curriculum/lin_vel_cmd_stability/failure_rate`, `track_ratio`, `episode_length_ratio`, and `range_max`.

For this variant, the curriculum expands the upper forward speed only. It does not keep lowering the minimum command toward standing, because this experiment is specifically about a running gait rather than one policy covering both standing and sprinting.

### Gait Cleanup Run From Model 20500

Checkpoint staging:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms_gait/from_sprint_model_20500/model_20500.pt`

Launch command:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Sprint-10ms-Gait \
  --resume \
  --load_run from_sprint_model_20500 \
  --checkpoint model_20500.pt \
  --run_name sprint10_gait_from_20500 \
  --max_iterations 10000
```

The run writes to:

`logs/rsl_rl/unitree_g1_29dof_sprint_10ms_gait/2026-05-14_21-28-49_sprint10_gait_from_20500/`

Launch log:

`logs/rsl_rl/unitree_g1_sprint10_gait_from_20500_20260514_212843.log`

Watcher log:

`logs/rsl_rl/unitree_g1_sprint10_gait_watch_20260514_212843.log`

The run resumed from `model_20500.pt` and targets iteration `30500`. Initial logged curriculum range is `4.0 m/s`. The watcher is monitoring `Curriculum/lin_vel_cmd_levels` and will stop the tmux training session only after the curriculum reaches `10.0 m/s` and a matching checkpoint exists.
