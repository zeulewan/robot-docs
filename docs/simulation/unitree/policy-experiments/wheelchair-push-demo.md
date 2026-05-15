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

## Passive Manual Chair Asset

On May 15, 2026, a manual wheelchair asset wrapper was added to `unitree_rl_lab`:

| Item | Value |
|---|---|
| Code commit | `6f8ca0f Add passive manual wheelchair asset wrapper` |
| URDF wrapper | `assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair.urdf` |
| Isaac Lab config | `source/unitree_rl_lab/unitree_rl_lab/assets/objects/wheelchair.py` |
| Import script | `scripts/assets/import_free3d_active_wheelchair.py` |
| Local visual mesh | `assets/objects/wheelchair/free3d_active_wheelchair/visual/active_wheelchair.obj` |

The source visual model is the Free3D active manual wheelchair (`active-wheelchair-82422`). The archive was copied locally from the Mac and normalized to meters: about `0.91 m` long, `0.67 m` wide, and `0.94 m` tall. The detailed OBJ/MTL/textures are intentionally ignored by Git because the fork is public and the Free3D page lists the model as personal-use.

The committed URDF is the physics source of truth. It uses primitive collisions for the chair body, passive rear wheel joints, passive front caster yaw/wheel joints, approximately `15.3 kg` total empty-chair mass, and fixed left/right handle frames near the push handles. The OBJ is visual-only.

## First Handle-Grip Run

Started on May 15, 2026:

```bash
source /home/zeul/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab
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

Early checkpoint:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_push/2026-05-15_03-34-51_wheelchair_push_from_walk_7200/model_7300.pt`

Early metrics around iteration `7305` were stable enough to keep training: mean episode length reached `1000`, `bad_orientation` was `0.0`, velocity-tracking reward was about `0.94`, and `hand_handle_position_l2` was down to about `0.0010`.

## Model 7300 Preview

Recorded on May 15, 2026 from the first handle-grip checkpoint:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-grip-model-7300.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_push/2026-05-15_03-34-51_wheelchair_push_from_walk_7200/model_7300.pt` |
| Demo output | `logs/demos/wheelchair_grip_model_7300_20260515_034201/rl-video-step-50.mp4` |
| Command | fixed `0.55 m/s` forward, zero lateral, zero yaw |
| Final logged displacement | `5.063 m` |

After recording the preview, training was resumed from `model_7300.pt` in tmux session `unitree_g1_wheelchair_push_train` with a target of about `model_12200.pt`.

## Stage-One Free3D Hand-Handle Playback

Recorded on May 15, 2026 as the first explicit hand-to-handle visual attachment demo using the downloaded Free3D wheelchair mesh:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-stage1-hand-connectors.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Script | `scripts/rsl_rl/play_wheelchair_push.py` |
| Visual mode | `--wheelchair-visual free3d` |
| Visual mesh | `assets/objects/wheelchair/free3d_active_wheelchair/visual/active_wheelchair.obj` |
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_push/2026-05-15_03-34-51_wheelchair_push_from_walk_7200/model_7300.pt` |
| Demo output | `logs/demos/wheelchair_free3d_stage1_20260515_050941/rl-video-step-50.mp4` |
| Command | fixed `0.55 m/s` forward, zero lateral, zero yaw |
| Final logged displacement | `5.142 m` |

This is still a playback-only stage-one demo. The script builds USD mesh prims from the locally downloaded Free3D OBJ, keeps that chair at the robot-relative handle offset, then draws world-level connector rods and endpoint markers between `left_wrist_yaw_link` / `right_wrist_yaw_link` and the chair handle target positions. That makes the intended attachment points visible in the video, but it is not yet a physical D6 constraint, contact model, or dynamic wheeled-chair training setup.

## Dynamic Wheelchair Push Training

Started on May 15, 2026 as the first actual physics version of the task:

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Dynamic-Push` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Reward helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/rewards.py` |
| Wheelchair asset | `assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair.urdf` |
| Code commit | `ee130c0 Add dynamic wheelchair push training task` |
| Output experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/` |
| Warm-start checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_push/2026-05-15_03-45-02_wheelchair_push_continue_model_7300/model_10000.pt` |

This variant puts the passive wheelchair articulation into the training scene instead of using the playback prop. The chair starts in front of the robot, its handle links now have small collision boxes, and the reward stack uses the moving wheelchair body positions instead of fixed robot-frame target points.

The policy observation and action shapes are still unchanged from the earlier walking and handle-grip runs: policy observation `(480,)`, critic observation `(495,)`, and action `(29,)`. That keeps the old checkpoint loadable. The wheelchair state is used for rewards and contact checks, but is not yet part of the policy observation.

The new task adds reward terms for keeping both wrist-yaw links near the real chair handles, matching the wheelchair's forward velocity to the command, rewarding forward wheelchair progress, penalizing lateral drift, yaw spin, and chair tilt, rewarding contact on the two handles, and penalizing robot contact on the chair body and wheels.

Smoke test command:

```bash
source /home/zeul/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --num_envs 1 \
  --max_iterations 1 \
  --run_name dynamic_wheelchair_smoke
```

The one-iteration smoke test completed after replacing broad PhysX contact filters with exact G1 body filters. The remaining expected warnings are that the wheelchair has passive joints with no actuators, plus noisy missing visual references for caster-yaw links that do not block training.

Training command:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run from_proxy_model_10000 \
  --checkpoint model_10000.pt \
  --run_name dynamic_push_from_proxy_10000 \
  --max_iterations 5000
```

Training tmux:

`unitree_g1_wheelchair_dynamic_push_train`

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_13-10-58_dynamic_push_from_proxy_10000/`

Launch log:

`logs/rsl_rl/unitree_g1_wheelchair_dynamic_push_from_proxy_10000_20260515_131051.log`

The run loaded `model_10000.pt` and started at RSL-RL iteration `10000/15000`.

This is a first version. If it learns too slowly, the next likely changes are to add wheelchair-relative handle observations to the policy, reduce the initial chair speed target, add a short grip/settle curriculum before pushing speed is rewarded, or temporarily lower chair mass/friction while the agent learns contact.
