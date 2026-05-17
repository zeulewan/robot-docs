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

The task changes the default arm pose so the rubber hand bodies start near a wheelchair handle position, then adds two rewards:

| Reward | Purpose |
|---|---|
| `hand_handle_position` | Exponential positive reward for keeping the left/right rubber hand bodies near fixed handle targets. |
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

## Training Collision Proxy Turntable

Recorded on May 15, 2026 to inspect the actual wheelchair proxy the dynamic push policies train against: the primitive collision/articulation model from the URDF, without the detailed Free3D visual mesh.

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/active-manual-wheelchair-training-collision-two-orbits.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Renderer | `scripts/assets/render_wheelchair_asset_turntable.py --mode training-collision` |
| Code commit | `9312b00 Render wheelchair training collision proxy` |
| URDF source | `assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair.urdf` |
| Docs asset | `docs/assets/active-manual-wheelchair-training-collision-two-orbits.mp4` |
| Output video | `logs/asset_turntables/active_manual_wheelchair_training_collision_two_orbits.mp4` |

The policy does not learn contact from the detailed OBJ visual mesh. It learns against this URDF articulation: the box seat/back/frame/handle collisions, rear-wheel cylinders, front-caster cylinders, passive joints, masses, and contact sensors. The turntable uses brighter inspection colors, but the shapes and poses come from the training URDF collision geometry.

```bash
TERM=xterm conda run -n isaaclab python scripts/assets/render_wheelchair_asset_turntable.py \
  --mode training-collision \
  --output logs/asset_turntables/active_manual_wheelchair_training_collision_two_orbits.mp4 \
  --width 1280 \
  --height 720 \
  --fps 24 \
  --duration 12 \
  --turns 2
```

## Free3D Asset Turntable

Recorded on May 15, 2026 to inspect the underlying downloaded wheelchair visual by itself, without the G1, contact helpers, or policy playback:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/free3d-active-wheelchair-asset-two-orbits.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Renderer | `scripts/assets/render_wheelchair_asset_turntable.py` |
| Code commit | `c542cfc Add wheelchair asset turntable renderer` |
| Visual mesh | `assets/objects/wheelchair/free3d_active_wheelchair/visual/active_wheelchair.obj` |
| Docs asset | `docs/assets/free3d-active-wheelchair-asset-two-orbits.mp4` |
| Output video | `logs/asset_turntables/free3d_active_wheelchair_two_orbits.mp4` |

The renderer is a lightweight software turntable for the normalized OBJ/MTL visual asset. It uses the same local Free3D visual mesh as the playback demos and renders two complete revolutions at `1280x720`, `24 fps`, and `12 s`.

```bash
TERM=xterm conda run -n isaaclab python scripts/assets/render_wheelchair_asset_turntable.py \
  --output logs/asset_turntables/free3d_active_wheelchair_two_orbits.mp4 \
  --width 1280 \
  --height 720 \
  --fps 24 \
  --duration 12 \
  --turns 2
```

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

The new task adds reward terms for keeping both rubber hand bodies near the real chair handles, matching the wheelchair's forward velocity to the command, rewarding forward wheelchair progress, penalizing lateral drift, yaw spin, and chair tilt, rewarding contact on the two handles, and penalizing robot contact on the chair body and wheels.

Correction added after the first dynamic preview: the original handle-contact reward accepted contact from any G1 body on the handle prims. Commit `a3d0222 Restrict wheelchair handle contact to hands` first narrowed the filter to wrist-yaw/rubber-hand end effectors. Commit `e7cc050 Use rubber hands for wheelchair handle rewards` then tightened the rule so only `left_rubber_hand` and `right_rubber_hand` are valid handle-contact bodies; wrist-yaw contact on a handle is now included in the invalid-contact penalty for new runs.

Smoke test for the rubber-hand-only rule:

```bash
TERM=xterm timeout 240s conda run -n isaaclab python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --num_envs 1 \
  --max_iterations 1 \
  --run_name rubber_hand_only_smoke
```

The smoke test completed and resolved the observed task with policy observation shape `(585,)`, critic observation shape `(600,)`, and the handle state still at `(60,)`, now computed from the two rubber-hand bodies.

After the code change, the old observed training process was stopped at the latest saved checkpoint `model_17600.pt` because it had been launched before the rubber-hand-only rule existed. Training was restarted from that checkpoint with a fresh optimizer state so the new reward/contact semantics are active:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --resume \
  --load_run 2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700 \
  --checkpoint model_17600.pt \
  --load_model_only \
  --run_name dynamic_push_observed_rubber_hands_resume_17600 \
  --max_iterations 3100
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-04-24_dynamic_push_observed_rubber_hands_resume_17600/`

Training tmux:

`unitree_g1_wheelchair_dynamic_push_observed_train`

Early restart status: the run loaded `left_rubber_hand` and `right_rubber_hand` for the handle observations/rewards, started at iteration `17600/20700`, and had `bad_orientation` at `0.0` in the first printed iterations. Reward initially dropped because the target body changed from wrist yaw links to rubber hands, so this continuation should be treated as an adaptation stage.

## Model 18000 Rubber-Hand Preview

Recorded and emailed on May 15, 2026 from the rubber-hand-only restart:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-rubber-hands-model-18000-latest.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-04-24_dynamic_push_observed_rubber_hands_resume_17600/model_18000.pt` |
| Demo output | `logs/demos/wheelchair_latest_20260515_222030/model_18000_latest.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-rubber-hands-model-18000-latest.mp4` |
| Sender script | `scripts/rsl_rl/send_latest_wheelchair_video.sh` |
| Script commit | `15d935c Add latest wheelchair video sender` |

This clip was produced with the repeatable sender script, which finds the latest checkpoint in the rubber-hand run, renders a short follow-camera playback, copies the MP4 to `logs/demos/`, and emails it with the local `gog` setup.

```bash
./scripts/rsl_rl/send_latest_wheelchair_video.sh
```

The script accepts overrides through environment variables such as `RUN_DIR`, `CHECKPOINT`, `EMAIL_TO`, `SEND_EMAIL=0`, `VIDEO_LENGTH`, and `VIDEO_CAMERA_ORBIT_DEG`. It intentionally does not store local keyring passwords or OAuth details.

For newer runs, prefer the general workstation tool documented in [Operator Tools](../../../infrastructure/operator-tools.md#isaac-playback-videos-isaac-clip):

```bash
isaac-clip send unitree-wheelchair-attached
```

That tool stores project/view presets in `~/.config/isaac-clip/projects.toml`, archives MP4 plus JSON metadata, and can use either the `gog` email provider or local file output.

## Model 18100 Rubber-Hand Two-Orbit Preview

Recorded and emailed on May 15, 2026 from the same rubber-hand-only restart, using a `720` degree follow-camera orbit so the camera revolves around the policy twice:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-rubber-hands-model-18100-two-orbits.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-04-24_dynamic_push_observed_rubber_hands_resume_17600/model_18100.pt` |
| Demo output | `logs/demos/wheelchair_latest_two_orbits_20260515_222542/model_18100_latest.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-rubber-hands-model-18100-two-orbits.mp4` |
| Email message id | `19e2e9c865f76694` |

Repeat command:

```bash
VIDEO_LENGTH=600 VIDEO_CAMERA_ORBIT_DEG=720 ./scripts/rsl_rl/send_latest_wheelchair_video.sh
```

## Wrist Alignment Restart

After reviewing the `model_18100` two-orbit video, the left wrist was visibly bending close to `90` degrees. That is likely a reward shortcut: the previous reward made `left_rubber_hand`/`right_rubber_hand` position and handle contact important, but did not directly penalize a rotated palm or wrist posture as long as the rubber-hand body reached the handle.

Code commit `8f85452 Penalize wheelchair wrist bend` adds two terms to the dynamic wheelchair task:

| Reward term | Purpose |
|---|---|
| `hand_handle_axis_alignment` | Penalizes the selected rubber-hand body axis being perpendicular to the wheelchair handle frame axis. |
| `wrist_joint_deviation` | Penalizes wrist roll, pitch, and yaw deviation from the configured handle-grip default pose. |

Smoke test:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --num_envs 1 \
  --max_iterations 1 \
  --run_name wrist_orientation_smoke
```

The smoke test resolved the observed task and showed `34` active reward terms, including the new `hand_handle_axis_alignment` and `wrist_joint_deviation` metrics. The policy observation remains `(585,)`, critic observation remains `(600,)`, and action shape remains `(29,)`, so the previous checkpoint can still warm-start the policy.

Training was restarted from `model_18300.pt` with a fresh optimizer state:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --resume \
  --load_run 2026-05-15_22-04-24_dynamic_push_observed_rubber_hands_resume_17600 \
  --checkpoint model_18300.pt \
  --load_model_only \
  --run_name dynamic_push_observed_wrist_alignment_resume_18300 \
  --max_iterations 2400
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-36-27_dynamic_push_observed_wrist_alignment_resume_18300/`

## Model 18700 Wrist-Alignment Preview

Recorded and emailed on May 15, 2026 from the wrist-alignment restart, using the same `720` degree two-orbit follow camera as the previous comparison clip:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-wrist-alignment-model-18700-two-orbits.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-36-27_dynamic_push_observed_wrist_alignment_resume_18300/model_18700.pt` |
| Demo output | `logs/demos/wheelchair_wrist_alignment_model_18700_20260515_225254/model_18700_latest.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-wrist-alignment-model-18700-two-orbits.mp4` |
| Email message id | `19e2eb5773838dbf` |

Render command:

```bash
CHECKPOINT=/home/zeul/GIT/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-36-27_dynamic_push_observed_wrist_alignment_resume_18300/model_18700.pt \
VIDEO_LENGTH=600 \
VIDEO_CAMERA_ORBIT_DEG=720 \
./scripts/rsl_rl/send_latest_wheelchair_video.sh
```

Original dynamic-task smoke test command:

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

## Model 10100 Dynamic Preview

Recorded and emailed on May 15, 2026 after the first dynamic-task checkpoint:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-dynamic-model-10100.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_13-10-58_dynamic_push_from_proxy_10000/model_10100.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_13-10-58_dynamic_push_from_proxy_10000/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-dynamic-model-10100.mp4` |
| Command | dynamic play task, `10` envs, fixed `0.45 m/s` forward command from the play config, follow-best camera |

The preview shows the actual dynamic wheelchair asset in the scene, not the earlier kinematic prop. This checkpoint is still very early; the chair is moving, but the gait/contact behavior needs more training and likely reward cleanup.

After recording the preview, training was restarted from `model_10100.pt` with `--max_iterations 4900`, targeting the same final iteration `15000`. That restart was then paused because the handle-contact rule was too loose.

After commit `a3d0222`, training was restarted again from `model_10100.pt` with hand-only handle contact:

```bash
source /home/zeul/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run 2026-05-15_13-10-58_dynamic_push_from_proxy_10000 \
  --checkpoint model_10100.pt \
  --run_name dynamic_push_hand_only_resume_10100 \
  --max_iterations 4900
```

## Model 11800 Hidden-Helper Dynamic Preview

Recorded and emailed on May 15, 2026 after the hand-only contact fix had trained past `model_11800.pt`. Commit `955aac8 Hide wheelchair helper visuals` removed the URDF primitive wheel/caster/handle visuals from rendered output while keeping the collision geometry active for training.

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-dynamic-hidden-helpers-model-11800.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_14-28-41_dynamic_push_hand_only_resume_11700/model_11800.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_14-28-41_dynamic_push_hand_only_resume_11700/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-dynamic-hidden-helpers-model-11800.mp4` |
| Command | dynamic play task, `10` envs, fixed `0.45 m/s` forward command from the play config, follow-best camera |

After recording the preview, training was restarted from `model_11800.pt` with `--max_iterations 3200`, targeting the same final iteration `15000`:

```bash
source /home/zeul/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run 2026-05-15_14-28-41_dynamic_push_hand_only_resume_11700 \
  --checkpoint model_11800.pt \
  --run_name dynamic_push_hidden_helpers_resume_11800 \
  --max_iterations 3200
```

## Model 14999 Final Dynamic Preview

Recorded and emailed on May 15, 2026 from the completed hidden-helper dynamic run:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-dynamic-final-model-14999.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_14-38-37_dynamic_push_hidden_helpers_resume_11800/model_14999.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_14-38-37_dynamic_push_hidden_helpers_resume_11800/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-dynamic-final-model-14999.mp4` |
| Command | dynamic play task, `10` envs, fixed `0.45 m/s` forward command from the play config, follow-best camera |

This is the completed `15000`-target run. It uses hand-only handle-contact rewards and hidden URDF helper visuals while keeping the collision geometry active for training.

## Four-Wheel Ground Bias

Added on May 15, 2026 after reviewing the `model_14999.pt` playback: the wheelchair front casters were lifting slightly while the robot pushed. Commit `2c5ca66 Add wheelchair four-wheel ground bias` changes both the asset and the reward stack:

| Change | Purpose |
|---|---|
| Lowered the front caster collision joint offset from `-0.05 m` to `-0.085 m` | Makes the front caster collision centers sit at `0.075 m`, so the front caster radius reaches the flat ground at reset instead of floating about `3.5 cm` above it. |
| Added `wheelchair_wheel_ground_height` reward term | Penalizes the four wheel/caster body centers drifting away from their nominal ground-contact heights. |
| Kept the existing `wheelchair_tilt` penalty | Still discourages global chair roll/pitch, while the new term catches practical caster unloading directly. |

The new reward uses the four wheelchair body names `left_rear_wheel`, `right_rear_wheel`, `left_front_caster`, and `right_front_caster`, with target body-center heights `[0.31, 0.31, 0.075, 0.075]` and a `1 cm` deadband. The smoke test successfully constructed the task and showed `wheelchair_wheel_ground_height` active in the reward table at weight `-50.0`.

Refinement training was started from the completed checkpoint:

```bash
source /home/zeul/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run 2026-05-15_14-38-37_dynamic_push_hidden_helpers_resume_11800 \
  --checkpoint model_14999.pt \
  --run_name dynamic_push_four_wheel_ground_resume_14999 \
  --max_iterations 2000
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999/`

Training tmux:

`unitree_g1_wheelchair_dynamic_push_train`

## Model 15200 Forward-Orbit Preview

Recorded and emailed on May 15, 2026 from the four-wheel ground-bias fine-tune:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-four-wheel-model-15200-forward-orbit.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999/model_15200.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-four-wheel-model-15200-forward-orbit.mp4` |
| Playback code commit | `d00af46 Add orbiting video follow camera` |
| Command | dynamic play task, `1` env, fixed `0.45 m/s` forward command, zero lateral/yaw command, follow camera with `25 deg` orbit |

This clip is intended as a cleaner single-agent visual check after the front-caster and wheel-height reward changes. The policy is still trained with the same observation/action shapes as the walking warm start; the wheelchair state is shaping rewards, not entering the policy observation.

## Straight-Line Bias

Added on May 15, 2026 after the `model_15200.pt` forward-orbit preview showed the robot/chair veering left. The command was still forward-only, so the left turn was policy drift rather than an intentional yaw command or just the camera orbit.

Commit `2ea2861 Add wheelchair straight-line bias` changes the dynamic reward stack in three ways:

| Change | Old | New |
|---|---:|---:|
| `wheelchair_lateral_velocity` | `-0.5` | `-2.0` |
| `wheelchair_yaw_velocity` | `-0.25` | `-1.0` |
| `wheelchair_forward_line` | not present | `-8.0` |

The new `wheelchair_forward_line` term penalizes wheelchair root lateral position away from the environment forward centerline, with a `5 cm` deadband. This complements the velocity penalties: lateral/yaw velocity discourages ongoing sideways and turning motion, while the forward-line term penalizes accumulated drift.

Smoke test:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --num_envs 1 \
  --max_iterations 1 \
  --run_name straightness_bias_smoke
```

The smoke test showed `32` reward terms, including `wheelchair_forward_line` at weight `-8.0`.

Straight-line fine-tune command:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run 2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999 \
  --checkpoint model_15700.pt \
  --run_name dynamic_push_straight_line_resume_15700 \
  --max_iterations 1300
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-04-58_dynamic_push_straight_line_resume_15700/`

Training tmux:

`unitree_g1_wheelchair_dynamic_push_train`

## Model 15800 Straight-Line Preview

Recorded and emailed on May 15, 2026 from the straight-line bias run:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-straight-line-model-15800-fixed-chase.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-04-58_dynamic_push_straight_line_resume_15700/model_15800.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-04-58_dynamic_push_straight_line_resume_15700/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-straight-line-model-15800-fixed-chase.mp4` |
| Command | dynamic play task, `1` env, fixed `0.45 m/s` forward command, zero lateral/yaw command, fixed chase camera with no orbit |

After `model_15800.pt`, the live training run became unstable: `bad_orientation` reached `1.0` and the forward rewards collapsed. Training was stopped before a `model_15900.pt` checkpoint was saved. This suggests the straight-line penalties were directionally useful but too aggressive as a hard continuation from `model_15700.pt`; the next version should soften the centerline term, ramp it in gradually, or add it through a short curriculum rather than applying the full `-8.0` weight immediately.

## Soft Straight-Line Continuation

Added on May 15, 2026 after reviewing the `model_15800.pt` fixed-chase video. The robot/chair still veered left and the gait looked wonky, but the aggressive straight-line continuation became unstable after `model_15800.pt`. Commit `40be05c Soften wheelchair straight-line penalties` keeps the same reward structure but reduces the straightness pressure:

| Reward | Aggressive | Soft |
|---|---:|---:|
| `wheelchair_lateral_velocity` | `-2.0` | `-1.0` |
| `wheelchair_yaw_velocity` | `-1.0` | `-0.5` |
| `wheelchair_forward_line` | `-8.0` | `-1.5` |

An attempted resume from `model_15800.pt` still showed high bad-orientation terminations early, so it was stopped. The active longer run resumes instead from the cleaner `model_15700.pt` checkpoint produced by the four-wheel ground-bias run:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push \
  --resume \
  --load_run 2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999 \
  --checkpoint model_15700.pt \
  --run_name dynamic_push_soft_straight_line_resume_15700 \
  --max_iterations 3000
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-15-18_dynamic_push_soft_straight_line_resume_15700/`

Launch log:

`logs/rsl_rl/unitree_g1_wheelchair_dynamic_push_soft_straight_line_resume_15700_20260515_181511.log`

Startup loaded `model_15700.pt`, showed the soft weights active, and had `bad_orientation` at `0.0` on the first printed iteration.

## Model 16000 Soft Straight-Line Preview

Recorded and emailed on May 15, 2026 from the soft straight-line run:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-soft-straight-line-model-16000-fixed-chase.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-15-18_dynamic_push_soft_straight_line_resume_15700/model_16000.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_18-15-18_dynamic_push_soft_straight_line_resume_15700/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-soft-straight-line-model-16000-fixed-chase.mp4` |
| Command | dynamic play task, `1` env, fixed `0.45 m/s` forward command, zero lateral/yaw command, fixed chase camera with no orbit |

The soft straight-line run was stopped after this checkpoint because the live training metrics had `bad_orientation` near `1.0`, indicating the continuation was no longer improving the gait. The current best path is likely not simply "train longer" with the same reward stack; it needs a gentler curriculum or better wheelchair/handle observations so the policy can correct veering without losing walking stability.

This is a first version. If it learns too slowly, the next likely changes are to add wheelchair-relative handle observations to the policy, reduce the initial chair speed target, add a short grip/settle curriculum before pushing speed is rewarded, or temporarily lower chair mass/friction while the agent learns contact.

## Observed Dynamic Push Variant

Added on May 15, 2026 after the `model_16000.pt` soft straight-line preview still veered left and then became unstable. The problem with the prior dynamic task was that the wheelchair was only used by rewards; the policy could not directly observe chair pose, chair velocity, chair heading, handle position, or hand-to-handle error. That made straight-line penalties hard to satisfy without damaging the walking gait.

The new task is registered as:

`Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed`

Code commit:

`bf09da1 Add observed wheelchair push task`

It adds these policy and critic observation terms:

| Term | Shape with history | Purpose |
|---|---:|---|
| `wheelchair_root_state` | `45` | Chair position, relative velocity, forward direction, yaw rate, and centerline error |
| `wheelchair_handle_state` | `60` | Left/right handle positions in robot-root frame plus hand-to-handle errors |

The old dynamic task remains at policy observation `(480,)` and critic observation `(495,)`. The observed task is policy observation `(585,)` and critic observation `(600,)`, so older checkpoints cannot be loaded directly. The first actor/critic input layer was expanded with zero-filled columns for the new observation terms:

```bash
TERM=xterm conda run -n isaaclab python scripts/rsl_rl/expand_input_checkpoint.py \
  logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push/2026-05-15_17-32-53_dynamic_push_four_wheel_ground_resume_14999/model_15700.pt \
  logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/from_dynamic_four_wheel_model_15700/model_15700.pt \
  --actor-input-dim 585 \
  --critic-input-dim 600
```

`model_16000.pt` was also expanded first, but that checkpoint already carried the collapsed gait from the soft straight-line run (`bad_orientation` around `1.0`), so it was abandoned. A first observed resume from `model_15700.pt` using the normal PPO settings also destabilized action updates, so the observed task was changed into a conservative adaptation stage:

| Setting | Value |
|---|---|
| Training command range | `0.15-0.35 m/s` forward only |
| Playback command | `0.3 m/s` forward only |
| PPO learning rate | `1e-4` |
| PPO clip | `0.1` |
| PPO epochs | `2` |
| PPO desired KL | `0.005` |

Active conservative run:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --resume \
  --load_run from_dynamic_four_wheel_model_15700 \
  --checkpoint model_15700.pt \
  --load_model_only \
  --run_name dynamic_push_observed_conservative_from_15700
```

Run folder:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700/`

Training tmux:

`unitree_g1_wheelchair_dynamic_push_observed_train`

Launch log:

`logs/rsl_rl/unitree_g1_wheelchair_dynamic_push_observed_conservative_from_15700_20260515_204700.log`

Early status: after the initial value/action-rate spike, the conservative run recovered by about iteration `15725`; value loss returned to normal, action-rate was around `-0.75`, reward was positive, and `bad_orientation` was around `0.20` rather than the `1.0` collapse seen in the `model_16000.pt` continuation.

## Model 16200 Observed Preview

Recorded and emailed on May 15, 2026 from the observed dynamic wheelchair-push run:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-observed-model-16200.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700/model_16200.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-observed-model-16200.mp4` |
| Command | observed dynamic play task, `1` env, fixed `0.3 m/s` forward command, zero lateral/yaw command, follow camera with slight `12 deg` orbit |

At the time of recording, the live training run had passed `model_16200.pt` and looked much healthier than the first observed attempts: reward was around `100`, `bad_orientation` had dropped to about `0.0`, and the wheelchair forward reward/contact terms were active. Yaw error was still nonzero, so this is an improved checkpoint rather than a final solved policy.

Working capture command:

```bash
latest_ckpt=$(find logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700 -maxdepth 1 -name 'model_*.pt' | sort -V | tail -1)

TERM=xterm conda run -n isaaclab python scripts/rsl_rl/play.py \
  --headless \
  --enable_cameras \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --num_envs 1 \
  --checkpoint "$latest_ckpt" \
  --video \
  --video-start-step 50 \
  --video_length 300 \
  --video-follow-robot \
  --video-camera-eye-offset -4.5 -3.2 2.2 \
  --video-camera-target-offset 0.5 0.0 0.9 \
  --video-camera-orbit-deg 12.0
```

Important capture note: using `--video` or `--enable_cameras` alone can still fail headless with `NO_GUI_OR_RENDERING`. Use both `--headless` and `--enable_cameras` for workstation offscreen video capture.

Emailing note: videos can be sent with the local `gog send` CLI and the existing keyring setup. Keep all local auth details out of this repo.

## Model 16300 Two-Orbit Preview

Recorded and emailed on May 15, 2026 from the observed dynamic wheelchair-push run with a two-revolution follow camera:

<video controls muted loop style="width: 100%; border-radius: 8px; margin: 1em 0;">
  <source src="../../../assets/g1-wheelchair-observed-model-16300-two-orbits.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700/model_16300.pt` |
| Demo output | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700/videos/play/rl-video-step-50.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-observed-model-16300-two-orbits.mp4` |
| Command | observed dynamic play task, `1` env, fixed `0.3 m/s` forward command, zero lateral/yaw command, `600` frames, `720 deg` follow-camera orbit |

This clip is `1280x720`, `50 fps`, and about `12 s` long. The camera orbit setting is `720 deg`, so the follow camera makes two complete revolutions around the tracked robot during the clip.

Two-orbit capture command:

```bash
latest_ckpt=$(find logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_20-47-07_dynamic_push_observed_conservative_from_15700 -maxdepth 1 -name 'model_*.pt' | sort -V | tail -1)

TERM=xterm conda run -n isaaclab python scripts/rsl_rl/play.py \
  --headless \
  --enable_cameras \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed \
  --num_envs 1 \
  --checkpoint "$latest_ckpt" \
  --video \
  --video-start-step 50 \
  --video_length 600 \
  --video-follow-robot \
  --video-camera-eye-offset -4.5 -3.2 2.2 \
  --video-camera-target-offset 0.5 0.0 0.9 \
  --video-camera-orbit-deg 720.0
```

## Attached-Hands Variant

On May 15, 2026, we started a new attached-hands version after seeing the left wrist bend sharply in the observed dynamic push run. The likely cause was that the policy was still learning to create handle contact through wrist/hand forces, so the wrist could become the easiest way to keep the chair moving.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Dynamic-Push-Attached` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Attachment helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/events.py` |
| Active run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_attached/2026-05-15_23-19-03_dynamic_push_attached_spherical_slow_from_19000` |
| Warm start | `unitree_g1_29dof_wheelchair_dynamic_push_observed/2026-05-15_22-36-27_dynamic_push_observed_wrist_alignment_resume_18300/model_19000.pt` |
| tmux | `unitree_g1_wheelchair_attached_train` |

Implementation notes:

The first attempt used hard `UsdPhysics.FixedJoint` constraints between `left_rubber_hand`/`right_rubber_hand` and the wheelchair handle frames. That made a closed loop between both arms and the wheelchair and the policy fought the constraint; the run quickly produced huge value loss and exited. The safer version uses `UsdPhysics.SphericalJoint` instead. That keeps each hand anchor on its matching handle while allowing the wrist/hand orientation to rotate freely.

For this attached variant, arm action scale is set to `0.0` for shoulder, elbow, and wrist joints while the leg/waist action scale stays at `0.25`. Robot joint reset velocities are set to zero, handle contact rewards are disabled because the hand-handle pair is collision-filtered, and `action_rate` is disabled for the transition run because the old observed policy was not trained with attached-hand constraints.

The first full restart still had nearly every environment terminate on `bad_orientation`, so the command range was reduced to `0.05-0.25 m/s` for the next restart. This is meant to let the policy relearn balance with the wheelchair constraint before asking it to push faster.

Current restart command:

```bash
TERM=xterm python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Push-Attached \
  --resume \
  --load_run warmstart_observed_19000 \
  --checkpoint model_19000.pt \
  --load_model_only \
  --run_name dynamic_push_attached_spherical_slow_from_19000 \
  --max_iterations 3000
```

Early status: the spherical-joint smoke test completed without the hard-joint numerical blow-up. The full slow-command run is active, but early iterations still show high `bad_orientation`, so this is not solved yet; the next checkpoint video should be judged mainly for whether the anchored hands stay on the handles and whether the robot starts recovering a stable walking gait.

## Attached Start-Pose Tuning

Added on May 16, 2026 after reviewing the attached-hands startup. The task reset itself was already deterministic: the robot and wheelchair reset x/y/yaw ranges were zeroed and reset velocities were zero. The remaining startup issue was geometric preload: the old default arm pose placed the G1 rubber-hand body origins about `4 cm` above the wheelchair handle frames and about `1.5 cm` behind them. Because the spherical hand-handle joint was created at startup, that offset made the visible hands and wrists start from a slightly artificial pose.

The fix keeps the wheelchair wheels on the ground and tunes the initial hand/chair geometry instead:

| Field | Previous | Updated |
|---|---:|---:|
| `wheelchair.init_state.pos.x` | `0.750` | `0.728` |
| handle target x in robot root frame | `0.350` | `0.328` |
| handle target z in robot root frame | `0.180` | `0.120` |
| left/right elbow default | `0.570` | `0.664` |
| left/right wrist pitch default | `-0.130` | `-0.088` |

The updated constants are in `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` as `WHEELCHAIR_HANDLE_TARGETS_B`, `WHEELCHAIR_ARM_JOINT_POSE`, and `DYNAMIC_WHEELCHAIR_INIT_POS`. The FK check after tuning put the rubber-hand origins within roughly `1 mm` of the URDF handle frames before the spherical joints are attached.

Short preview with the tuned startup geometry:

<video controls width="100%">
  <source src="../../../assets/g1-wheelchair-attached-start-pose-tuned-model-20100-preview.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_push_attached/2026-05-15_23-19-03_dynamic_push_attached_spherical_slow_from_19000/model_20100.pt` |
| Demo output | `logs/demos/start_pose_tuned_preview/unitree-wheelchair-attached_model_20100_short_best_20260516_000639/model_20100_short_best.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-attached-start-pose-tuned-model-20100-preview.mp4` |

This change affects new play/training processes. The training process that was already running when the edit was made still had the previous task config loaded in memory.

## Standing-First Attached Pretrain

Added on May 16, 2026 after the latest attached-hands playback showed the robot could not reliably stand still with the wheelchair constraint. The previous attached task still requested slow forward motion and kept walking-shaped incentives such as gait and foot clearance. That made the policy solve balance, hand anchoring, and pushing at the same time.

The new pretrain stage isolates the first skill:

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Attached` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_stand_attached/` |
| Warm start target | observed wheelchair checkpoint `model_19000.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |

Standing changes:

| Area | Change |
|---|---|
| Command | Set base velocity command to exactly zero: no forward, lateral, or yaw motion. |
| Locomotion rewards | Keep zero-velocity tracking, alive, upright orientation, base height, feet contact, and smooth action terms. |
| Walking incentives | Disable gait and foot-clearance rewards so the policy is not encouraged to step just to earn gait reward. |
| Wheelchair | Reward zero chair forward velocity and penalize chair XY/yaw velocity, centerline drift, tilt, and wheel/caster height error. |
| Randomization | Disable added torso mass during this first balance phase. |
| Action scale | Reduce leg/waist action scale to `0.12` and keep arm action scale at `0.0` so the first stand phase cannot flail as aggressively. |
| PPO update | Use a cold fine-tune: learning rate `1e-6`, clip `0.01`, one learning epoch, entropy `0.0`, max grad norm `0.05`, and no action-rate reward. |

The intended curriculum is: train this standing task until `bad_orientation` stays low and the video shows quiet balance, then use that checkpoint to warm-start the attached push task again. This is a pretrain stage, not the final wheelchair-pushing objective.

An initial attempt warm-started from the latest attached-push checkpoint, `model_21999.pt`, but `bad_orientation` quickly climbed to about `0.9985`. That checkpoint had already adapted toward an unstable pushing behavior, so it was stopped and the standing phase was switched back to the earlier observed checkpoint.

The first observed-checkpoint attempt started with `bad_orientation` around `0.17`, then collapsed to about `0.99` after the first PPO update. The cause was optimizer shock: the objective changed from moving/pushing to zero-motion standing, while the action-rate reward produced very large penalties. The standing runner now disables that reward and uses a smaller PPO update.

The attached constraint still proved too hard at reset. The next lower curriculum rung was `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed`: same wheelchair scene and same observed policy shape, but no spherical hand-handle joint yet. That stage starts stable but still drifts into `bad_orientation` when warm-started from the push-adapted `model_19000.pt`; it is too much of an objective change from pushing to standing.

The lower rung added after that is a plain standing task:

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Stand` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_stand/` |
| Active run | `logs/rsl_rl/unitree_g1_29dof_stand/2026-05-16_02-45-11_stand_from_walk_7200_cold/` |
| Warm start | walking checkpoint `logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/model_7200.pt` |
| Cold warm-start copy | `logs/rsl_rl/unitree_g1_29dof_stand/warmstart_walk_7200_cold/model_7200.pt` |
| First saved checkpoint | `logs/rsl_rl/unitree_g1_29dof_stand/2026-05-16_02-45-11_stand_from_walk_7200_cold/model_7250.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/standing_env_cfg.py` |
| tmux | `unitree_g1_stand_train` |

This task removes the wheelchair entirely, commands exactly zero base velocity, disables gait and foot-clearance rewards, freezes the arm action scale, keeps only low leg/waist action authority, and adds the same explicit fall penalty for `bad_orientation` and `base_height`. The cold checkpoint copy preserves the walking policy weights but lowers action noise to `0.02`.

Initial status from the plain standing run was healthy: by `model_7250`, `bad_orientation = 0.0`, `base_height = 0.0`, `fall_termination = 0.0`, and episode length reached the full `10 s` horizon. The run was stopped at `model_8150.pt`; that checkpoint is a stabilizer only, not the wheelchair target policy.

The first attempt to move `model_8150.pt` into `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed` still collapsed after a few PPO updates. The issue was not simply that the chair existed. That task also inherits the handle-arm reset pose from the push task while standing freezes arm actions, so the policy was asked to absorb a large arm-pose change and the wheelchair scene at the same time.

The corrected second rung is `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed-Neutral`. It keeps the wheelchair articulation and wheelchair observations, restores the base G1 neutral arm reset pose, keeps arm actions frozen, and disables handle/contact shaping for this bridge stage. That makes it "standing with the wheelchair present" before the handle pose or spherical hand-handle attachment is reintroduced.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed-Neutral` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_stand_observed_neutral/` |
| Active run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_stand_observed_neutral/2026-05-16_03-34-54_stand_observed_neutral_from_plain_stand_8150/` |
| Warm start | expanded plain stand checkpoint `model_8150.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| tmux | `unitree_g1_wheelchair_stand_neutral_train` |

As of the May 16, 2026 launch from expanded `model_8150.pt`, the neutral wheelchair-observed run reached full 500-step episodes with `bad_orientation = 0.0`, `base_height = 0.0`, and `fall_termination = 0.0` through the first several PPO updates. The run completed at `model_9649.pt`, and playback showed the robot standing with the wheelchair present but with neutral arms, not holding the handles. That makes it a stability bridge only.

Two direct next-step attempts failed. `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed` reintroduced the handle arm pose but no hand-handle joint; by `model_9700.pt`, `bad_orientation` was about `0.9985`. `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Attached` attached the hands to the dynamic wheelchair handles, but the free chair was pulled/tipped through the hand constraints and `bad_orientation` quickly rose to about `0.99`.

The fixed-base hold was stopped as an active training direction. It is useful as a diagnostic because the hand-handle spherical joints are less explosive when the wheelchair root cannot move, but it can teach the wrong shortcut: the robot may learn to stand by loading the handles into an immovable chair instead of balancing its own body. That is not the primitive we want to transfer into a real free wheelchair.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Fixed-Stand-Attached` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_stand_attached/` |
| Diagnostic run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_stand_attached/2026-05-16_11-13-17_fixed_stand_attached_from_neutral_9649/` |
| Warm start | neutral wheelchair-observed checkpoint `model_9649.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Code commit | `ee3a5d0 Add fixed-base wheelchair hold task` |

Early fixed-base status was better than the free-chair attached run but not solved. The first saved checkpoint was `model_9650.pt`; by iteration `9709`, about half of the environments were timing out, `bad_orientation` had dropped to about `0.4565`, and the wheelchair velocity/tilt rewards were zero because the chair was fixed for this rung.

Two free-wheelchair alternatives were tried next so the chair could stay physically movable while rewards biased it to remain stationary:

| Task ID | Change | Status |
|---|---|---|
| `Unitree-G1-29dof-Wheelchair-Stationary-Stand-Attached` | Free wheelchair, hands attached, strong root position/height/heading, velocity, tilt, and wheel-ground penalties. | Stopped; it still collapsed from the neutral standing checkpoint with `bad_orientation` near `0.99`. |
| `Unitree-G1-29dof-Wheelchair-Braked-Stationary-Stand-Attached` | Same stationary rewards plus wheel/caster velocity drives and light root damping. | Stopped; the braked free chair also collapsed early, with `bad_orientation` above `0.996`. |

Those failures suggest the current jump is too large: stable neutral standing plus wheelchair observations is not enough to immediately add the handle arm pose, attached hands, and a free chair.

The next bridge removed the wheelchair entirely and asked the robot to stand with the full handle-reaching arm pose. There was no wheelchair support and no hand-handle joint, so the policy had to balance its own body. That full pose was still too abrupt: it loaded the plain standing checkpoint, but after a few PPO updates `bad_orientation` climbed to about `0.94`, so the run was stopped.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Stand-Handle-Arms` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_stand_handle_arms/` |
| Stopped run | `logs/rsl_rl/unitree_g1_29dof_stand_handle_arms/2026-05-16_11-29-54_stand_handle_arms_from_plain_8150/` |
| Warm start | plain standing checkpoint `logs/rsl_rl/unitree_g1_29dof_stand/2026-05-16_02-45-11_stand_from_walk_7200_cold/model_8150.pt` |
| Warm-start copy | `logs/rsl_rl/unitree_g1_29dof_stand_handle_arms/from_plain_stand_8150/model_8150.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/standing_env_cfg.py` |
| Code commit | `e3092b7 Add handle-arm standing bridge` |
| tmux | stopped `unitree_g1_stand_handle_arms_train` |

The active replacement is `Unitree-G1-29dof-Stand-Reach-Arms`. It uses the same no-chair standing task, but the arm reset pose is only `35%` of the way from the neutral G1 arm pose to the full handle pose. This keeps the important constraint from the fixed-base discussion: the robot cannot lean on a fixed chair. It also turns the posture change into a smaller curriculum step before trying the full handle pose again.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Stand-Reach-Arms` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_stand_reach_arms/` |
| Stopped run | `logs/rsl_rl/unitree_g1_29dof_stand_reach_arms/2026-05-16_11-34-15_stand_reach_arms_from_plain_8150/` |
| First saved checkpoint | `logs/rsl_rl/unitree_g1_29dof_stand_reach_arms/2026-05-16_11-34-15_stand_reach_arms_from_plain_8150/model_8200.pt` |
| Last used checkpoint | `logs/rsl_rl/unitree_g1_29dof_stand_reach_arms/2026-05-16_11-34-15_stand_reach_arms_from_plain_8150/model_8400.pt` |
| Warm start | plain standing checkpoint `logs/rsl_rl/unitree_g1_29dof_stand/2026-05-16_02-45-11_stand_from_walk_7200_cold/model_8150.pt` |
| Warm-start copy | `logs/rsl_rl/unitree_g1_29dof_stand_reach_arms/from_plain_stand_8150/model_8150.pt` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/standing_env_cfg.py` |
| Code commit | `4a29cc5 Add partial arm reach standing bridge` |
| tmux | `unitree_g1_stand_reach_arms_train` |

Early status was better than the full handle-arm run: at the first saved checkpoint, `model_8200.pt`, normal timeouts were about `0.81` and `bad_orientation` was about `0.17`. The run was stopped at `model_8400.pt` when the next test moved back to the actual wheelchair-attached standing setup.

The immediate wheelchair-attached retry used `Unitree-G1-29dof-Wheelchair-Braked-Stationary-Stand-Attached`, not the fixed-base chair. The `model_8400.pt` reach-arm checkpoint was expanded from `(480, 495)` inputs to `(585, 600)` inputs for the wheelchair-observed policy/critic shape, then loaded with `--load_model_only`.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Braked-Stationary-Stand-Attached` |
| Run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_braked_stationary_stand_attached/2026-05-16_11-48-35_braked_stationary_stand_attached_from_reach_8400/` |
| Expanded checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_braked_stationary_stand_attached/from_reach_arms_8400/model_8400.pt` |
| Video | `logs/demos/unitree-wheelchair-braked-stand-attached_model_8400_startup_failure_20260516_115010/model_8400_startup_failure.mp4` |
| tmux | stopped `unitree_g1_wheelchair_braked_stand_attached_train` |

This run still collapsed immediately with the hand-handle attachment active. Iteration `8400` already had short episodes and `bad_orientation` around `0.57`; by iteration `8401`, `bad_orientation` was about `0.998`, so the run was stopped. The current blocker is therefore not visualization: the free wheelchair and hand attachment are present, but the attached starting condition is still too hard for the policy.

After that, commit `b0f32d9 Add relaxed wheelchair attached stand variants` added two diagnostic variants. `Unitree-G1-29dof-Wheelchair-Relaxed-Stand-Attached` keeps the braked dynamic wheelchair and hand-handle attachment, but restores arm action authority instead of freezing the arms. It also greatly relaxes the chair root, wheel-ground, velocity, and wrist-deviation penalties so the robot is not immediately punished for every small chair movement. `Unitree-G1-29dof-Wheelchair-Left-Hand-Relaxed-Stand-Attached` is the same diagnostic idea, but attaches only the left rubber hand to check whether the closed-chain two-hand constraint is the main source of startup preload.

| Item | Value |
|---|---|
| Two-hand task ID | `Unitree-G1-29dof-Wheelchair-Relaxed-Stand-Attached` |
| Left-hand task ID | `Unitree-G1-29dof-Wheelchair-Left-Hand-Relaxed-Stand-Attached` |
| Two-hand reach-arm run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_stand_attached/2026-05-16_12-02-06_relaxed_stand_attached_from_reach_8400/` |
| Left-hand reach-arm run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_left_hand_relaxed_stand_attached/2026-05-16_12-05-07_left_hand_relaxed_stand_attached_from_reach_8400/` |
| Two-hand neutral run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_stand_attached/2026-05-16_12-06-40_relaxed_stand_attached_from_neutral_9649/` |
| Sent preview | `logs/demos/unitree-wheelchair-relaxed-stand-attached_model_9650_startup_failure_20260516_120937/model_9650_startup_failure.mp4` |
| Config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Code commit | `b0f32d9 Add relaxed wheelchair attached stand variants` |

All three relaxed diagnostics were stopped. The two-hand run from the reach-arm checkpoint failed immediately with `bad_orientation` around `0.995+`. The one-hand diagnostic was slightly less violent, around `0.94-0.95`, but still failed. Retrying the relaxed two-hand task from the stable neutral wheelchair-observed checkpoint `model_9649.pt` also failed immediately, with `bad_orientation` near `0.9995` at the first update.

The takeaway is that this is not just an arm-freezing mistake or an overly tight reward stack. Freeing the arms and relaxing the rewards did not recover standing, and the stable neutral wheelchair checkpoint also collapses once the hard hand-handle attachment is added. The next fix should target the attachment mechanics themselves: inspect the spherical USD joint frames, avoid reset preload, consider creating or enabling the hand-handle constraint after the robot has reached the handles, or replace the hard startup joint with a softer staged constraint.

Passive ragdoll diagnostic recorded on May 16, 2026:

<video controls preload="metadata" style="max-width: 100%;">
  <source src="../../../assets/g1-wheelchair-relaxed-attached-ragdoll-startup.mp4" type="video/mp4">
</video>

| Item | Value |
|---|---|
| Video | `logs/demos/unitree-wheelchair-relaxed-stand-attached-ragdoll_20260516_122238/model_9650_ragdoll_startup.mp4` |
| Docs asset | `docs/assets/g1-wheelchair-relaxed-attached-ragdoll-startup.mp4` |
| Playback commit | `de2b8b1 Add diagnostic ragdoll playback flags` |
| Diagnostic flags | `--zero-actions --ragdoll-robot --disable-fall-terminations` |

This playback uses the same relaxed attached scene, but does not use the policy. The robot is made passive by zeroing joint stiffness and damping, actions are zeroed, and the fall/base-height terminations are disabled so the clip shows the raw startup physics instead of immediately resetting.

The free-chair relaxed continuation was stopped on May 16, 2026 after it still showed no useful recovery. Around iteration `10851`, `bad_orientation` was about `0.9999`, timeout was effectively zero, and mean episode length was only about `5.1` steps. That supported the earlier diagnosis that the free wheelchair plus hard hand attachment remained too unstable for this curriculum step.

The next diagnostic is `Unitree-G1-29dof-Wheelchair-Fixed-Relaxed-Stand-Attached`, added in commit `c4d019a Add fixed-base relaxed wheelchair stand task`. It locks the wheelchair root like the earlier fixed-base diagnostic, but keeps the relaxed arm and wrist action scales from the free-chair relaxed task instead of freezing the arms. This is intentionally a scaffold: it may teach some support against an immovable handle, but it isolates whether the policy can learn the hand-held standing pose when the wheelchair base is not moving under it.

| Item | Value |
|---|---|
| Task ID | `Unitree-G1-29dof-Wheelchair-Fixed-Relaxed-Stand-Attached` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/` |
| Active run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/2026-05-16_14-39-39_fixed_relaxed_stand_attached_from_neutral_9649/` |
| Warm start | neutral wheelchair-observed standing checkpoint `model_9649.pt` |
| Warm-start copy | `logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/from_neutral_stand_9649/model_9649.pt` |
| Code commit | `c4d019a Add fixed-base relaxed wheelchair stand task` |
| tmux | `unitree_g1_wheelchair_fixed_relaxed_stand_attached_train` |

Early fixed-base relaxed status is significantly better than the free-chair relaxed run. At iteration `9668`, `bad_orientation` was about `0.455`, `base_height` was near zero, timeout was about `0.545`, mean episode length was about `365` steps, and mean reward was positive at about `29.1`. This is not a final transferable solution because the chair root is fixed, but it is a useful bridge checkpoint candidate if it continues improving.

While reviewing the fixed-base preview, we noticed a visible two-step startup: the hands appear off the handles at the start of the clip, then snap onto the handles shortly after physics begins. A follow-up playback with direct hand-origin to handle-origin anchoring produced PhysX warnings that the hand-handle joints were created with disjoint body transforms. The measured no-attachment reset offset was almost entirely vertical: both handle frames were about `0.04029 m` below the rubber-hand origins. The wheelchair URDF handle frames were raised from `z=0.88` to `z=0.92029`, after which the reset error dropped to about `0.0005 m` per hand and the attached-task startup check no longer emitted the disjoint-joint snap warning. The active attached tasks now create direct origin-to-origin spherical joints after reset instead of preserving a startup offset.

For visual inspection of the simplified URDF itself, `play.py` now has a diagnostic playback flag, `--show-wheelchair-urdf-proxy`. It swaps the normal Free3D visual mesh for `assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair_proxy_visual.urdf`, which renders the URDF proxy boxes/cylinders for the seat, frame, wheels, casters, and handle frames. This is a playback-only view for checking what the training collision/handle model looks like; the normal training asset remains `active_manual_wheelchair.urdf`.

The verified slow-revolve proxy clip is currently the newest video on the fixed latest-video dashboard:

```text
https://workstation.tailee9084.ts.net:8002/
```

Archived video:

```text
logs/demos/unitree-wheelchair-urdf-proxy_model_11100_slow_revolve_20260516_170039/model_11100_urdf_proxy_slow_revolve.mp4
```

After reviewing that proxy render, the handle collision boxes were found to overlap the backrest collision in the side lanes where the hands attach. Each handle overlapped the backrest by roughly `6.5 cm` in X, `3.75 cm` in Y, and `4.7 cm` in Z. That meant the hard hand-handle attachment could pull the hand into the chair back collision, which looked like the hands were fighting the seat/back rather than simply holding the handles.

The immediate collision-clearance fix narrowed the seat collision from `0.48 x 0.46 x 0.08` to `0.48 x 0.38 x 0.08`, narrowed the backrest collision from `0.07 x 0.50 x 0.44` to `0.07 x 0.32 x 0.44`, and shrank each handle collision box from `0.18 x 0.055 x 0.055` to `0.12 x 0.04 x 0.04`. The handle frame origins stayed at `z=0.92029`, so the hand-handle startup alignment stayed at about `0.5 mm` instead of reintroducing the old snap.

| Item | Value |
|---|---|
| Resume checkpoint | `model_11250.pt` |
| Restarted tmux | `unitree_g1_wheelchair_fixed_relaxed_stand_attached_collision_clearance_train` |
| New run name | `fixed_relaxed_stand_attached_collision_clearance_from_11250` |
| Preview video | `logs/demos/unitree-wheelchair-collision-clearance_model_11250_slow_revolve_20260516_172016/model_11250_collision_clearance_slow_revolve.mp4` |

Follow-up on May 16, 2026: the fixed-base relaxed attached diagnostic was changed to test a straighter wrist posture. The hand-handle attachment collision mask was disabled for this temporary diagnostic, so the rubber hand and handle can collide instead of being filtered. The wrist deviation reward now targets explicit zero wrist roll/pitch/yaw instead of the reset/default wrist pose, the fixed-base relaxed task strengthens that term to `-0.25`, and wrist action authority is reduced from `0.03` to `0.015`. This should bias the learned hold toward straight wrists while still allowing small corrections.

After the fixed-base attached standing policy looked stable, training moved into a walking/pushing phase. The new task is `Unitree-G1-29dof-Wheelchair-Relaxed-Push-Attached`. It keeps the hard spherical hand-to-handle attachments and the relaxed arm action scales from the successful standing diagnostic, but switches the wheelchair back to a free dynamic root and restores forward wheelchair velocity/progress rewards. The point is to warm-start from the stable attached stand instead of asking a cold policy to solve standing, hand constraints, and chair motion at the same time.

| Item | Value |
|---|---|
| Walking task | `Unitree-G1-29dof-Wheelchair-Relaxed-Push-Attached` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_push_attached/` |
| Active run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_push_attached/2026-05-16_19-21-50_relaxed_push_attached_from_fixed_stand_12250/` |
| Warm start | fixed-base relaxed attached standing `model_12250.pt` |
| Warm-start copy | `logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_push_attached/warmstart_fixed_relaxed_stand_12250/model_12250.pt` |
| Training tmux | `unitree_g1_wheelchair_relaxed_push_attached_train` |
| Auto-preview tmux | `isaac_clip_watch_wheelchair_walk_250` |

The active auto-preview watcher updates the latest-video site every `250` training iterations:

```bash
isaac-clip watch unitree-wheelchair-relaxed-push-attached \
  --target-iteration 12500 \
  --every-iterations 250 \
  --view two_orbit \
  --render \
  --notify \
  --to <email>
```

At setup time the new walking run had produced `model_12300.pt`, so the first automatic preview target was `model_12500.pt`. The watcher then advances to `model_12750.pt`, `model_13000.pt`, and so on. The watcher uses the `site` provider, so it replaces the embedded video and sends a short `gog` notification email after each successful render:

```text
https://workstation.tailee9084.ts.net:8002/
```

On May 16, 2026, the walking continuation stopped at `model_12650.pt` before reaching the next watcher target. The trainer failed with `RuntimeError: normal expects all elements of std >= 0.0`, so the auto-preview watcher was stopped instead of waiting forever for `model_12750.pt`.

The same review found that the G1 `left_rubber_hand` and `right_rubber_hand` links from the Unitree ROS URDF had visuals and inertials but no collision geometry. That made the task look like the wrists were doing the physical work: the spherical joints targeted the rubber-hand body names, but the hand link origin is at the palm joint and the rubber hand itself had no collider. The fix generates a temporary G1 URDF under `/tmp/IsaacLab/unitree_rl_lab/g1_29dof_hand_collision/` with small palm collision boxes added to both rubber-hand links, without modifying the upstream `unitree_ros` checkout.

The attachment point was also moved from the hand body origin to a measured palm grip offset. `HAND_GRIP_LOCAL_POSITIONS` is now approximately `[0.05414, +/-0.00372, 0.00502]` in each rubber-hand frame, and the wheelchair root x offset moved from `0.728` to `0.782` so the handle frames line up with those palm points. The policy observation and hand/handle reward helpers were updated to use the same palm grip point instead of the raw rubber-hand origin. A one-env playback diagnostic reported about `5e-6 m` handle-to-grip error after reset, while the raw handle-to-hand-origin offset remained about `0.0545 m`; that confirms the visible attachment point is now the palm/hand, not the wrist-side body origin.

The continuation PPO settings were also made more conservative for the next walking run: learning rate `1e-5`, desired KL `5e-4`, entropy coefficient `0.001`, and max grad norm `0.1`. The one-iteration smoke run constructs the task and reaches `Learning iteration 0/1`; PhysX still prints a hand-handle joint warning during reset, so preview videos should continue to be checked for startup snap, but the measured reset alignment is now at the palm grip point rather than the wrist/link origin. Follow-up commit `a5c4bce` adds `play.py --print-hand-handle-offsets` output for the actual USD joint `localPos0`, which should match the measured palm offset instead of a wrist-side or off-hand location.

The first palm-grip walking restart from `model_12500.pt` was stopped around iteration `12516`: `bad_orientation` was climbing toward `0.18` and `wheelchair_invalid_contact` had grown past `-200`. That pointed to a new collision-proxy issue after adding palm collisions. Commit `f295fc7` shrinks the generated rubber-hand collision box from `0.090 x 0.060 x 0.040 m` to `0.070 x 0.035 x 0.030 m` and centers it at the measured palm area, reducing accidental contact with the wheelchair side-frame/base proxy while keeping an actual hand collider.

After that, the `model_12500.pt` continuation was abandoned because it had already adapted to the old hand-origin attachment. The active replacement resumes from the cleaner fixed-base relaxed attached standing checkpoint:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Relaxed-Push-Attached \
  --resume \
  --load_run warmstart_palm_grip_fixed_stand_12250 \
  --checkpoint model_12250.pt \
  --load_model_only \
  --run_name relaxed_push_attached_palm_grip_from_fixed_stand_12250
```

Active run:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_push_attached/2026-05-16_22-40-59_relaxed_push_attached_palm_grip_from_fixed_stand_12250/`

Early status is better than the `12500` restart but not solved: by about iteration `12258`, `bad_orientation` was about `0.125` and `wheelchair_invalid_contact` was about `-128`. The preview watcher now targets `model_12500.pt` for this new run.

By `model_12650.pt`, the reward scalars still looked superficially encouraging for forward tracking, with `Episode_Reward/wheelchair_track_forward_velocity` staying around `1.0`, but a raw Isaac playback diagnostic showed the chair was not actually moving forward well. The diagnostic sampled `400` steps across `10` environments with the playback command fixed at `0.18 m/s`; the wheelchair forward velocity averaged `-0.033 m/s`, ranged from `-0.512` to `0.374 m/s`, and was within `0.10 m/s` of the command only about `9%` of the time. Lateral absolute velocity averaged `0.058 m/s` and yaw absolute velocity averaged `0.425 rad/s`. Treat this run as unstable/poor for actual pushing speed despite the reward printout.

Raw velocity diagnostic command:

```bash
conda run --no-capture-output -n isaaclab python scripts/rsl_rl/play.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Relaxed-Push-Attached \
  --num_envs 10 \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_relaxed_push_attached/2026-05-16_22-40-59_relaxed_push_attached_palm_grip_from_fixed_stand_12250/model_12650.pt \
  --print-wheelchair-speed-stats \
  --speed-stats-steps 400 \
  --show-wheelchair-urdf-proxy
```

The next check separated wheelchair physics from the learned policy. `scripts/diagnostics/wheelchair_force_check.py` creates the same task, disables the hand-handle attachment event, resets the robot `5 m` behind the chair, disables robot fall resets, applies a constant world-X force to selected wheelchair bodies, and prints raw chair motion. This checks whether the wheelchair asset itself can roll straight before tuning rewards or arm constraints.

Base-link force check:

```bash
conda run --no-capture-output -n isaaclab python scripts/diagnostics/wheelchair_force_check.py \
  --headless \
  --force 10 0 0 \
  --force-body base_link \
  --steps 400 \
  --num-envs 10
```

Result: the wheelchair rolled forward and stayed straight. A `10 N` push on `base_link` produced mean forward velocity `0.0367 m/s`, lateral absolute velocity `0.0002 m/s`, yaw absolute velocity `0.0007 rad/s`, and final mean displacement `0.303 m` with only `0.0009 m` mean lateral drift.

Symmetric handle force check:

```bash
conda run --no-capture-output -n isaaclab python scripts/diagnostics/wheelchair_force_check.py \
  --headless \
  --force 5 0 0 \
  --force-body '.*handle_frame' \
  --steps 400 \
  --num-envs 10
```

Result: applying `5 N` to each handle frame, for the same `10 N` total forward force, produced essentially the same motion: mean forward velocity `0.0366 m/s`, lateral absolute velocity `0.0002 m/s`, yaw absolute velocity `0.0007 rad/s`, and final mean displacement `0.303 m`.

This means the passive wheelchair asset can roll straight when pushed cleanly. The poor policy result is therefore more likely from the two-hand closed-chain attachment, robot-chair collisions, and reward balance than from a basic wheel/caster physics failure.

A visual check was rendered with a stronger symmetric handle push so the motion is easy to see on the latest-video page:

```bash
conda run --no-capture-output -n isaaclab python scripts/diagnostics/wheelchair_force_check.py \
  --headless \
  --video \
  --force 15 0 0 \
  --force-body '.*handle_frame' \
  --steps 400 \
  --num-envs 1
```

The rendered clip is served through the latest-video site:

```text
https://workstation.tailee9084.ts.net:8002/
```

The video run applied `15 N` to each handle frame. It produced mean forward velocity `0.112 m/s`, lateral absolute velocity `0.0028 m/s`, yaw absolute velocity `0.0071 rad/s`, and final displacement `0.926 m` with about `0.024 m` lateral drift.

Plain standing launch:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Stand \
  --resume \
  --load_run warmstart_walk_7200_cold \
  --checkpoint model_7200.pt \
  --load_model_only \
  --run_name stand_from_walk_7200_cold \
  --max_iterations 1500
```

To move a plain standing checkpoint into the wheelchair-observed neutral standing task, expand its first actor/critic input layers to the observed dimensions and keep the added observation columns zero-initialized:

```bash
python scripts/rsl_rl/expand_input_checkpoint.py \
  logs/rsl_rl/unitree_g1_29dof_stand/<stand-run>/model_<iter>.pt \
  logs/rsl_rl/unitree_g1_29dof_wheelchair_dynamic_stand_observed_neutral/from_plain_stand_<iter>/model_<iter>.pt \
  --actor-input-dim 585 \
  --critic-input-dim 600
```

Neutral observed standing launch:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed-Neutral \
  --resume \
  --load_run from_plain_stand_<iter> \
  --checkpoint model_<iter>.pt \
  --load_model_only \
  --run_name stand_observed_neutral_from_plain_stand_<iter> \
  --max_iterations 1500
```

Partial-reach bridge launch:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --task Unitree-G1-29dof-Stand-Reach-Arms \
  --resume \
  --load_run from_plain_stand_8150 \
  --checkpoint model_8150.pt \
  --load_model_only \
  --run_name stand_reach_arms_from_plain_8150 \
  --max_iterations 1500
```
