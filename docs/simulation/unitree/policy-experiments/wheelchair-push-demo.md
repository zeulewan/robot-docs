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
