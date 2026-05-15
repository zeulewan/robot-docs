# G1 Wheelchair Push Demo

Quick demo requested on May 15, 2026: use the existing walking policy, put a wheelchair-like prop in front of the G1, and make the pair move forward.

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-push-demo.mp4" type="video/mp4">
</video>

## Setup

| Item | Value |
|---|---|
| Script | `scripts/rsl_rl/play_wheelchair_push.py` |
| Code commit | `8fc1b29 Add wheelchair push playback demo` |
| Policy task | `Unitree-G1-29dof-Velocity` |
| Policy checkpoint | `logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/model_7200.pt` |
| Command | fixed `0.55 m/s` forward, zero lateral, zero yaw |
| Scene | one G1 on a flat plane |
| Video | `logs/demos/wheelchair_push_20260515_013315/rl-video-step-50.mp4` |

The script creates a simple wheelchair from USD primitives and keeps it at a fixed robot-relative offset in front of the G1. It also adds lower rear grip bars so the shot reads like the robot is connected to the chair handle area. The walking policy itself is unchanged.

Recorded displacement from the run log:

| Step | Robot XY displacement |
|---:|---:|
| `100` | `0.918 m` |
| `200` | `1.987 m` |
| `300` | `3.029 m` |
| `400` | `4.099 m` |
| `500` | `5.147 m` |
| final | `5.841 m` |

## Command

```bash
TERM=xterm python scripts/rsl_rl/play_wheelchair_push.py \
  --headless \
  --video \
  --video-start-step 50 \
  --video_length 500 \
  --video-folder logs/demos/wheelchair_push_20260515_013315 \
  --task Unitree-G1-29dof-Velocity \
  --num_envs 1 \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/model_7200.pt \
  --command-x 0.55 \
  --camera-mode fixed
```

## Caveat

This is a visualization demo, not a physically valid wheelchair-pushing policy. The chair is kinematically attached to the walking rollout; it is not a dynamic wheeled articulation, there is no hand/handle constraint, no contact reward, and the chair state is not in the policy observation. A real version should define an articulated wheelchair asset, constrain or contact the handle, add chair-displacement and stability rewards, and train or fine-tune the policy for pushing.

## Handle-Grip Training Task

On May 15, 2026, a first trainable wheelchair-push proxy was added.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Push` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Reward helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/rewards.py` |
| Code commit | `4b23515 Add G1 wheelchair push training task` |
| Output experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_push/` |
| Warm-start checkpoint | `logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/model_7200.pt` |
| Warm-start copy | `logs/rsl_rl/unitree_g1_29dof_wheelchair_push/from_walk_model_7200/model_7200.pt` |

This variant keeps the policy observation and action spaces identical to the base walking task: policy observation shape is still `(480,)`, critic observation shape is still `(495,)`, and the action shape is still `29`. That is deliberate. It lets the wheelchair task resume from the walking checkpoint instead of relearning the whole walking gait.

The task changes the default arm pose so the wrist-yaw links start near a wheelchair handle position, then adds two rewards:

| Reward | Purpose |
|---|---|
| `hand_handle_position` | Exponential positive reward for keeping the left/right wrist-yaw links near fixed handle targets. |
| `hand_handle_position_l2` | L2 penalty for the same hand-handle error. |

The current handle targets are expressed in the robot root frame:

```python
[
    [0.35, 0.24, 0.18],
    [0.35, -0.24, 0.18],
]
```

The trainable proxy still does not simulate a real rolling wheelchair constraint. It trains the robot to walk forward with its hands held at the handle locations. The next step is to connect this trained posture to a dynamic wheelchair asset or fixed/d6 hand-handle constraints so pushing force and chair motion are part of the physics.

## First Handle-Grip Run

Started on May 15, 2026:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Push \
  --resume \
  --load_run from_walk_model_7200 \
  --checkpoint model_7200.pt \
  --run_name wheelchair_push_from_walk_7200 \
  --max_iterations 5000
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_push/2026-05-15_03-34-51_wheelchair_push_from_walk_7200/`

Training tmux:

`unitree_g1_wheelchair_push_train`

Training log:

`logs/rsl_rl/unitree_g1_wheelchair_push_from_walk_7200_20260515_033445.log`

Because this resumes from `model_7200.pt`, RSL-RL reports the run as iteration `7200` onward and the `--max_iterations 5000` target corresponds to about `model_12200.pt`.
