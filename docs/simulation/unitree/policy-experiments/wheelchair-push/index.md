# G1 Wheelchair Push Policy

This page is the working summary for the wheelchair-push policy experiments. Keep current decisions here and move detailed chronological notes to the [archive](archive.md).

## Current Status

| Item | Value |
|---|---|
| Active training task | `Unitree-G1-29dof-Wheelchair-Minimal-FreeYaw-GroundLock-1mps-Fast-Lean-HeavyDamping-Push-Attached-Hard` |
| Last rendered playback | free-yaw heavy-damping bridge: `model_19300.pt`, rendered May 23 at 13:27 Toronto |
| Main config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Observation helpers | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py` |
| Attachment helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/events.py` |
| Phase 1 source checkpoint | `model_19247.pt` from the good 1 m/s PhysX-rail hard-attach run |
| Preserved 2 m/s visual reference | `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-Fast-Lean-Velocity-Progress-Push-Attached-Hard`, `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_fast_lean_hard_attach_push_attached/2026-05-18_19-47-36_hard_attach_loose_guard_2048_from_13249/model_13300.pt` |
| Current run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_freeyaw_groundlock_1mps_fast_lean_heavydamp_hard_attach_push_attached/2026-05-23_*_freeyaw_heavydamp_from_19247_may23` |
| Failed branch kept for comparison | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_hard_attach_push_attached/2026-05-18_20-34-46_hard_1mps_yawtorque_from_fastlean_13300` |
| Latest archived playback | `logs/demos/unitree-wheelchair-freeyaw-groundlock-1mps-fast-lean-heavydamp-hard-attach-push-attached_model_19300_slow_revolve_best_20260523_132740/model_19300_slow_revolve_best.mp4` |
| Summary last updated | May 23, 2026, 13:34 Toronto |
| Training tmux | `freeyaw_heavydamp_train`; latest-video site still running |
| Training env count | `2048` |
| Latest-video page | `https://workstation.tailee9084.ts.net:8002/` |
| Focused TensorBoard | `http://workstation.tailee9084.ts.net:6007/` |

The previous soft-attachment run was stopped at `model_15900.pt` and used as the baseline for the SoftObs branch. That branch is not the visually good reference. The visually useful 2 m/s hard-attachment fast-lean lineage is preserved as the visual reference and warm-start source. The earlier 1 m/s yaw-torque hard branch did not learn useful forward push behavior and is kept only as a failed comparison branch. The current 1 m/s run is a new speed-scaled variant of the reproducible 2 m/s path, not that yaw-torque branch.

Current status: the May 22 rail-free transfer attempts and the first May 23 fixed-chair scratch bridge were useful diagnostics, but they are no longer the active plan. The fixed-chair path learned to stand, then failed as soon as X/Y/yaw were released, which means it likely taught the robot to lean on an artificial support. Phase 1 was rewritten as a ground-up damping-release curriculum, but the first Phase 1A run also failed: by roughly `model_700-800`, `bad_orientation` was effectively `1.0`, `time_out` was `0.0`, and mean episode length was only about `10-12` steps. The run was stopped before the `model_2499` gate because it was not producing usable PPO rollouts.

Working diagnosis: hard hand-handle attachment from a scratch/random policy is too large a first step. The reset pose and chair pose place the hands near the handles, but the policy cannot keep the constrained humanoid upright long enough to collect useful standing data. The next branch should restore a viable bridge before the damping-release ladder: either warm-start from the stable fixed-relaxed attached standing checkpoint, start from a reach/stand primitive and introduce the hard hand-handle constraint after that policy is stable, or use a softer/staged grip before switching to hard attachment.

Important correction from the May 19 git-history audit and reproduction test: treat the `model_13300.pt` hard-attach checkpoint as a real good visual reference, not as a randomly fragile checkpoint. The exact old command from the `model_13249.pt` source reproduced `model_13300.pt` and `model_13350.pt` byte-for-byte on May 19. The behavior changed in the later restart/modified branches, while the original 2 m/s hard-attach path is still reproducible.

## May 23 Rigid-To-Free Bridge

The new active path is not the failed scratch Phase 1A run. It starts from the successful 1 m/s hard-attach PhysX-rail actor and removes the forward rail gradually. The first bridge keeps hard hand-handle attachment and the wheelchair ground-plane lock, leaves forward X motion free, and damps only the unstable lateral/yaw axes.

Current task:

`Unitree-G1-29dof-Wheelchair-Minimal-FreeYaw-GroundLock-1mps-Fast-Lean-HeavyDamping-Push-Attached-Hard`

Training command:

```bash
TERM=xterm conda run --no-capture-output -n isaaclab python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Minimal-FreeYaw-GroundLock-1mps-Fast-Lean-HeavyDamping-Push-Attached-Hard \
  --run_name freeyaw_heavydamp_from_19247_may23 \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_15-22-06_hard_attach_1mps_continue_full_from_18248_may19/model_19247.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.03 \
  --max_iterations 500
```

Bridge settings:

| Axis | Setting |
|---|---|
| Forward X velocity | free, `x_velocity_scale = 1.0` |
| Lateral Y velocity | heavily damped, `y_velocity_scale = 0.05` |
| Chair yaw velocity | heavily damped, `yaw_velocity_scale = 0.05` |
| Ground plane | height, roll, and pitch constrained so the chair stays on the floor |
| Rewards added beyond the rail task | lateral velocity, forward-line drift, yaw velocity, root heading, forward-heading penalties |

Early signal at iteration `19252`: unlike the scratch release, the bridge produced useful rollouts. `bad_orientation` was below `0.001`, `base_height` was `0.0`, unstable robot/chair resets were `0.0`, and wheelchair forward progress was nonzero.

The first bridge checkpoint, `model_19300.pt`, was rendered at `13:27 Toronto` and published to the latest-video site. Training remained live during render. Around iteration `19333`, the training process was still healthy: `time_out` was about `0.986`, `bad_orientation` about `0.014`, `base_height` `0.0`, `unstable_robot_state` `0.0`, `unstable_wheelchair_state` `0.0`, and `wheelchair_forward_progress` about `0.87`. This is not a solved free-chair push yet, but it is a viable non-rigid bridge branch compared with the failed scratch damping-release run.

Playback also confirmed that the actor policy observation vector is getting robot joint state and chair state. For this task, policy obs shape is `(585,)`, including `joint_pos_rel`, `joint_vel_rel`, `last_action`, `base_ang_vel`, `projected_gravity`, `velocity_commands`, `wheelchair_root_state`, and `wheelchair_handle_state`. In other words, the immediate issue is not that PPO is missing the robot joint angles; the harder part is transitioning from rail/rigid support to a chair that can drift and yaw without destabilizing the learned push.

The latest-video site was restarted at `13:24 Toronto` so the New Video button targets `unitree-wheelchair-freeyaw-groundlock-1mps-fast-lean-heavydamp-hard-attach-push-attached`.

## May 22 Free-Yaw Ground-Lock Playback

The next diagnostic asks whether the best 1 m/s hard-attach policy can still push when the chair is not held on the PhysX X/yaw rail. This is playback-only for now, not a training branch.

Task added in `unitree_rl_lab` commit `661b663`:

`Unitree-G1-29dof-Wheelchair-Minimal-FreeYaw-GroundLock-1mps-Fast-Lean-Push-Attached-Hard`

The task uses the regular no-collision wheelchair URDF instead of `active_manual_wheelchair_x_rail.urdf`. It keeps the hard hand-handle attachment and the same policy/reward shape as the 1 m/s reference, but the only wheelchair constraint event is:

`constrain_wheelchair_to_ground_plane`

That event pins wheelchair height, roll, and pitch every environment step, while preserving X, Y, yaw, and yaw velocity. In other words: all four wheels are effectively kept on the floor, but the chair is free to drift sideways or rotate if the policy pushes off-center.

Playback command:

```bash
isaac-clip send unitree-wheelchair-freeyaw-groundlock-1mps-fast-lean-hard-attach-push-attached \
  --view slow_revolve_best \
  --checkpoint /home/zeul/GIT/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_15-22-06_hard_attach_1mps_continue_full_from_18248_may19/model_19247.pt \
  --run-dir /home/zeul/GIT/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_15-22-06_hard_attach_1mps_continue_full_from_18248_may19 \
  --provider site \
  --training-policy continue \
  --no-compress-site-video
```

Rendered output:

`logs/demos/unitree-wheelchair-freeyaw-groundlock-1mps-fast-lean-hard-attach-push-attached_model_19247_slow_revolve_best_20260522_045657/model_19247_slow_revolve_best.mp4`

The Isaac event table for this playback showed `constrain_wheelchair_to_ground_plane` as the only interval event; there was no `constrain_wheelchair_to_forward_rail` event and no PhysX X-rail asset in the task. The latest-video site was restarted with this project as the New Video target.

Status after inspecting the playback and running raw speed stats: the policy does not transfer cleanly once yaw is free. It still produces intermittent chair motion, but it does not walk straight or track the `1.0 m/s` wheelchair command. Over `900` steps and `10` envs, measured wheelchair `base_link` forward speed averaged only `0.0183 m/s`, with `0.8%` of samples within `0.10 m/s` of the command. Mean absolute yaw rate was `0.8466 rad/s`, mean absolute lateral speed was `0.3206 m/s`, and the final centerline offset averaged `0.3932 m`. Treat the rail-free playback as a failed transfer diagnostic: the current actor is relying heavily on the rail/yaw constraint.

Operational note: the first render attempt failed before environment creation because the workstation had an NVIDIA driver/library mismatch after an update (`580.126.09` kernel module loaded with `580.159.03` user-space libraries). The final successful render was made after rebooting into the matching `580.159.03` driver. Future whole-workstation reboots should be cleared with the team first.

## Rail-Free Curriculum Task List

Rule for this curriculum: each phase stops at its gate. Do not roll straight into the next phase. At each gate, render a playback to the latest-video site, send a Telegram notification, write the status here, then wait for review before continuing.

Use `isaac-clip` for the review gate. The normal shape is:

```bash
isaac-clip send <phase-project> \
  --view slow_revolve_best \
  --provider site \
  --training-policy continue
```

For automatic checkpoint gates inside a phase:

```bash
isaac-clip watch <phase-project> \
  --every-iterations <interval> \
  --view slow_revolve_best \
  --render \
  --notify \
  --notify-provider telegram
```

The Telegram update should say which phase just completed or which checkpoint was rendered, link the latest-video page, and include the checkpoint path. Do not put bot tokens, chat IDs, credentials, or keyring details in docs.

For the rewritten damping-release ladder, use the gated auto-advancer instead of manually chaining each phase:

```bash
tmux new-session -d -s wheelchair_phase_advancer -c /home/zeul/GIT/unitree_rl_lab \
  'tools/wheelchair_phase_advancer.py \
    --start-phase phase1a \
    --current-run-dir logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_phase1a_damped_release_directobs/2026-05-23_06-50-27_scratch_phase1a_damped_release_no_forward_reward_from_zero_may23 \
    --poll-seconds 60 \
    --render \
    --notify \
    --notify-provider telegram \
    --codex-review \
    2>&1 | tee logs/rsl_rl/wheelchair_phase_advancer_may23.log'
```

The auto-advancer is not a blind phase jumper. At each gate it waits for the phase checkpoint, renders the latest-video site, sends the gate result through Telegram, reads TensorBoard scalars, and advances only if the health gate passes. The default Phase 1 gate requires low `bad_orientation`, low `base_height`, and high `time_out`. If the gate fails, it stops instead of launching the next phase.

With `--codex-review`, the gate also launches a read-only `codex exec` reviewer. The reviewer gets a JSON context bundle with the phase config, checkpoint, TensorBoard metrics, latest-video path/metadata, this docs page, and the task config. It must return structured JSON with `decision: advance`, `stop`, or `needs_human`. The hard metric gate still vetoes advancement, and Codex can also veto or require human review. Codex writes a concise Telegram update in the JSON; the parent advancer sends that update with `telecli send --agent isaac`, so the child Codex process does not directly mutate files, stop training, or send messages itself. Email is only a fallback if Telegram is unavailable.

- [x] Phase 0: build the rail-free DirectObs scaffold. Use the regular wheelchair URDF, hard hand-handle attachment, no X/yaw rail, and the ground-plane lock only for wheelchair height, roll, and pitch.
- [ ] Phase 1A: train from zero with chair ground lock and planar velocity damping at `x/y/yaw = 0.0/0.0/0.0`. This is the current run. It should learn stable standing/holding without any forward-push reward.
- [ ] Phase 1B: continue from Phase 1A with heavy chair damping, `0.05/0.02/0.02`. The chair can move a little, but the dynamics should still be easy.
- [ ] Phase 1C: continue with medium chair damping, `0.20/0.10/0.10`.
- [ ] Phase 1D: continue with light chair damping, `0.60/0.35/0.35`.
- [ ] Phase 1E: continue with no artificial planar damping, `1.0/1.0/1.0`, while still keeping the chair on the ground plane. Phase 1 ends only when this free-standing hold is visually stable.
- [ ] Phase 2: add a tiny forward push only after Phase 1E passes. Start around `0.15-0.25 m/s` and keep yaw/lateral/centerline controls conservative.
- [ ] Phase 3: slow walking push at `0.4-0.6 m/s`. Add light gait/smoothness shaping only after forward motion is clearly working.
- [ ] Phase 4: straight `1.0 m/s` push without the rail. Success means the wheelchair tracks near `1.0 m/s` without spinning or side-slipping.
- [ ] Phase 5: robustness. Randomize small wheelchair yaw/position offsets, handle offsets, friction/damping, and command speed while staying straight-only.
- [ ] Phase 6: turning later. Add yaw/turn commands only after straight pushing works. Do not add turning early, because it will make spinning/side-pushing easier to rediscover.

Minimum gate metrics for Phases 3-6:

| Metric | Early accept target |
|---|---:|
| Wheelchair forward speed | clearly positive and trending toward the command |
| Samples within `0.10 m/s` of command | improving phase to phase; not near zero |
| Wheelchair yaw absolute mean | low enough that the chair is not visibly spinning |
| Wheelchair lateral absolute speed | low enough that the chair is not side-slipping |
| Centerline final offset | small and bounded over the playback |
| Robot health | no sustained falling/crawling/flailing |

### May 22 Phase 1 Start

Task added in `unitree_rl_lab` commit `99caba8`:

`Unitree-G1-29dof-Wheelchair-RailFree-Phase1-Stand-Attached`

This is the first trainable rail-free scaffold. It uses the regular no-collision wheelchair asset, hard hand-handle attachment, and the `constrain_wheelchair_to_ground_plane` interval event. The event pins chair height, roll, and pitch, but does not pin X, Y, or yaw. The reward stack has no forward-progress reward in this phase; it is a standing/holding gate with zero commanded velocity and penalties for chair lateral motion, yaw rate, root displacement, and root heading.

Training command shape:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-RailFree-Phase1-Stand-Attached \
  --max_iterations 250 \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/2026-05-16_18-06-04_fixed_relaxed_stand_attached_straight_wrists_from_11600/model_12300.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.01 \
  --run_name railfree_phase1_groundlock_from_fixed_relaxed_12300_may22
```

The intended gate watcher was:

```bash
isaac-clip watch unitree-wheelchair-railfree-phase1-stand-attached \
  --target-iteration 12550 \
  --view slow_revolve_best \
  --render \
  --provider site \
  --training-policy continue \
  --notify \
  --notify-provider telegram
```

The run saved `model_12549.pt` as its final checkpoint, so the watcher waiting for `12550` did not fire. The stale watcher was stopped and the gate was rendered manually from `model_12549.pt`:

```bash
isaac-clip watch unitree-wheelchair-railfree-phase1-stand-attached \
  --target-iteration 12549 \
  --run-dir logs/rsl_rl/unitree_g1_29dof_wheelchair_railfree_phase1_stand_attached/2026-05-22_06-13-39_railfree_phase1_groundlock_from_fixed_relaxed_12300_may22 \
  --view slow_revolve_best \
  --render \
  --provider site \
  --training-policy continue \
  --notify \
  --notify-provider telegram
```

Rendered output:

`logs/demos/unitree-wheelchair-railfree-phase1-stand-attached_model_12549_slow_revolve_best_20260522_200012/model_12549_slow_revolve_best.mp4`

This gate should be treated as a failed Phase 1 attempt. Final train metrics around `12549/12550` had `Episode_Termination/bad_orientation` near `0.87`, `wheelchair_xy_velocity` around `-0.79`, and `wheelchair_root_position` around `-3.83`, so the stationary no-rail stand setup is not yet holding the chair cleanly.

### May 22 Phase 1 PoseObs Test

The follow-up hypothesis was that the hard hand-handle constraint made the task partially observable: the old `wheelchair_handle_state_b` only exposed handle positions and hand-to-handle position error, so once the spherical joints held the hands on the handles, the policy could be blind to twist and angular motion at the grip.

Added code:

- `wheelchair_hand_handle_pose_state_b` in `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py`
- `DynamicWheelchairPushHardAttachmentPoseObservationsCfg` in `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py`
- `Unitree-G1-29dof-Wheelchair-RailFree-Phase1-PoseObs-Stand-Attached`

The new observation term adds, for both hand-handle pairs, the wheelchair handle basis vectors expressed in the hand frame plus relative handle angular velocity expressed in the hand frame. With the existing history stack of `5`, it adds `120` policy inputs. The resulting shapes are policy `(705,)`, critic `(720,)`, action `(29,)`.

The fixed-relaxed standing-with-handles checkpoint was expanded so the old actor can warm-start while ignoring the new inputs initially:

```bash
python scripts/rsl_rl/expand_input_checkpoint.py \
  logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/2026-05-16_18-06-04_fixed_relaxed_stand_attached_straight_wrists_from_11600/model_12300.pt \
  logs/rsl_rl/expanded_checkpoints/fixed_relaxed_stand_attached_model_12300_poseobs_705_720.pt \
  --actor-input-dim 705 \
  --critic-input-dim 720
```

Training command:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-RailFree-Phase1-PoseObs-Stand-Attached \
  --max_iterations 250 \
  --resume \
  --checkpoint logs/rsl_rl/expanded_checkpoints/fixed_relaxed_stand_attached_model_12300_poseobs_705_720.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.01 \
  --run_name railfree_phase1_poseobs_from_fixed_relaxed_12300_may22
```

The run was stopped at the first adapted checkpoint instead of burning the full gate because the failure appeared immediately. By `model_12350.pt`, `Episode_Termination/bad_orientation` was still about `0.87`, `wheelchair_xy_velocity` was around `-0.76`, and `wheelchair_root_position` was around `-3.46`. The rendered playback also failed visually: several robots were collapsed or teetering in the chairs.

Rendered output:

`logs/demos/unitree-wheelchair-railfree-phase1-poseobs-stand-attached_model_12350_slow_revolve_best_20260522_222657/model_12350_slow_revolve_best.mp4`

Conclusion: the missing hard-attachment orientation/angular-velocity observation was real, but adding it did not fix the current rail-free Phase 1 scaffold. The next experiment should change the physical scaffold or reward target: for example, use a more stationary support stage, simplify/disable the wheelchair forward command during standing, delay the hard attachment until a settled pose, or add an explicit attachment-load signal instead of relying only on relative pose.

### May 22 Phase 1 DirectObs Test

The next shortcut was to make the task more demo-oriented and expose the wheelchair state directly instead of asking the policy to infer it through robot proprioception. A second goal was to test whether a usable handle load signal exists.

Added code:

- `wheelchair_direct_root_state_b` in `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py`
- `wheelchair_hand_handle_wrench_state_b` in `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py`
- `DynamicWheelchairPushDirectStateObservationsCfg`
- `Unitree-G1-29dof-Wheelchair-RailFree-Phase1-DirectObs-Stand-Attached`

`wheelchair_direct_root_state_b` adds the wheelchair orientation basis expressed in the robot root frame plus wheelchair linear and angular velocity. `wheelchair_hand_handle_wrench_state_b` adds scaled incoming force/torque wrench on the two robot hand bodies and the two wheelchair handle bodies. With history length `5`, those two new terms add `75 + 120 = 195` policy inputs on top of PoseObs. The resulting shapes are policy `(900,)`, critic `(915,)`, action `(29,)`.

The handle wrench signal was checked before training. In an 8-env zero-action rollout, `wheelchair_hand_handle_wrench_state_b` had mean absolute scaled value about `0.326`, max `1.0` after clipping, and about `0.50` of entries were nonzero. That means the body incoming-wrench signal is alive and can be used as a load cue, even if it is not a named cross-asset joint reaction sensor.

The fixed-relaxed standing-with-handles checkpoint was expanded again:

```bash
python scripts/rsl_rl/expand_input_checkpoint.py \
  logs/rsl_rl/unitree_g1_29dof_wheelchair_fixed_relaxed_stand_attached/2026-05-16_18-06-04_fixed_relaxed_stand_attached_straight_wrists_from_11600/model_12300.pt \
  logs/rsl_rl/expanded_checkpoints/fixed_relaxed_stand_attached_model_12300_directobs_900_915.pt \
  --actor-input-dim 900 \
  --critic-input-dim 915
```

Training command:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-RailFree-Phase1-DirectObs-Stand-Attached \
  --max_iterations 250 \
  --resume \
  --checkpoint logs/rsl_rl/expanded_checkpoints/fixed_relaxed_stand_attached_model_12300_directobs_900_915.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.01 \
  --run_name railfree_phase1_directobs_from_fixed_relaxed_12300_may22
```

The run was stopped after the first saved checkpoint because the failure pattern was clear. Around `model_12350.pt`, `Episode_Termination/bad_orientation` was about `0.88`, `wheelchair_xy_velocity` was about `-0.80`, and `wheelchair_track_forward_velocity` remained tiny at about `0.03`. The rendered playback remained bad.

Rendered output:

`logs/demos/unitree-wheelchair-railfree-phase1-directobs-stand-attached_model_12350_slow_revolve_best_20260522_233201/model_12350_slow_revolve_best.mp4`

Conclusion: direct wheelchair state and handle/body wrench observations are technically working, but they still do not make the current rail-free Phase 1 learn. The next change should simplify the physical scaffold: for example, remove the forward-velocity command from the stand gate, start from a settled attached pose, reintroduce temporary damping/rails as a curriculum variable, or train a stationary hold with fewer competing wheelchair penalties before asking for rail-free pushing.

### May 23 Rewritten Phase 1

The first May 23 scratch bridge used a fixed wheelchair root, then tried to release the chair directly. It looked stable while fixed, but both release attempts failed quickly: the direct handoff from `model_300.pt` reached `bad_orientation` around `0.57`, and the low-load handoff from `model_500.pt` reached `bad_orientation` above `0.62`. Treat the fixed-chair Phase 1 and Phase 1B runs as deprecated diagnostics, not as the current curriculum.

The active replacement is a ground-up damping-release ladder. It keeps the same DirectObs policy shape throughout: policy `(900,)`, critic `(915,)`, action `(29,)`. It uses the ground-plane lock for chair height/roll/pitch and an interval event, `damp_wheelchair_planar_velocity`, to scale wheelchair X, Y, and yaw velocity every env step. Phase 1 has no forward-push reward: `wheelchair_track_forward_velocity = 0.0`, `wheelchair_forward_progress = 0.0`, and `wheelchair_backward_velocity = 0.0`.

| Stage | Task | Damping scales `x/y/yaw` | Goal |
|---|---|---:|---|
| 1A | `Unitree-G1-29dof-Wheelchair-Scratch-Phase1A-DampedRelease-DirectObs` | `0.0 / 0.0 / 0.0` | Learn attached standing from zero with the chair nearly immobile. |
| 1B | `Unitree-G1-29dof-Wheelchair-Scratch-Phase1B-HeavyDamping-DirectObs` | `0.05 / 0.02 / 0.02` | Allow tiny chair motion under heavy damping. |
| 1C | `Unitree-G1-29dof-Wheelchair-Scratch-Phase1C-MediumDamping-DirectObs` | `0.20 / 0.10 / 0.10` | Continue the same hold with medium damping. |
| 1D | `Unitree-G1-29dof-Wheelchair-Scratch-Phase1D-LightDamping-DirectObs` | `0.60 / 0.35 / 0.35` | Continue with light damping. |
| 1E | `Unitree-G1-29dof-Wheelchair-Scratch-Phase1E-FreeStand-DirectObs` | `1.0 / 1.0 / 1.0` | Stand/hold with no artificial planar damping. |

Active Phase 1A command:

```bash
TERM=xterm conda run --no-capture-output -n isaaclab python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Scratch-Phase1A-DampedRelease-DirectObs \
  --run_name scratch_phase1a_damped_release_no_forward_reward_from_zero_may23
```

Run directory:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_phase1a_damped_release_directobs/2026-05-23_06-50-27_scratch_phase1a_damped_release_no_forward_reward_from_zero_may23`

Training tmux:

```bash
tmux attach -t scratch_phase1a_train
```

Auto-advancer tmux:

```bash
tmux attach -t wheelchair_phase_advancer
```

Early verification after the fix: the smoke run showed `wheelchair_track_forward_velocity: 0.0000`, and the live 2048-env run also showed `wheelchair_track_forward_velocity: 0.0000` at the first logged updates. The earlier short run from `06:45` was discarded because it still had the old zero-velocity tracking reward active, which made the Phase 1 reward stack less clean than intended.

## Current Task Shape

The active task is the rewritten scratch Phase 1A, not the old PhysX X-rail push task. The wheelchair asset is the regular no-collision manual wheelchair URDF, and the task uses two interval events:

| Event | Purpose |
|---|---|
| `constrain_wheelchair_to_ground_plane` | Keeps the chair height, roll, and pitch fixed so the wheels stay on the floor. |
| `damp_wheelchair_planar_velocity` | Scales wheelchair X, Y, and yaw velocity as the Phase 1 curriculum variable. |

The active nonzero chair-specific reward terms in Phase 1A are:

| Reward | Weight | Purpose |
|---|---:|---|
| `wheelchair_lateral_velocity` | `-1.5` | Penalize side motion while learning to hold the chair. |
| `wheelchair_forward_line` | `-1.0` | Penalize leaving the forward centerline. |
| `wheelchair_yaw_velocity` | `-1.5` | Penalize twisting the chair. |
| `robot_hand_wrench` | `-0.02` | Discourage leaning too hard through the robot hand bodies. |
| `wheelchair_handle_wrench` | `-0.01` | Discourage excessive load at the wheelchair handle bodies. |
| `wheelchair_root_heading` | `-1.0` | Penalize chair yaw away from the reset heading. |
| `wheelchair_xy_velocity` | `-1.5` | Penalize chair planar velocity. |
| `wheelchair_root_position` | `-2.0` | Penalize chair root drift. |

Forward-push terms are deliberately off in Phase 1A: `wheelchair_track_forward_velocity`, `wheelchair_forward_progress`, and `wheelchair_backward_velocity` all have weight `0.0`.

## Observation Gap

The policy observes the robot state plus wheelchair-relative state through `wheelchair_root_state_b` and `wheelchair_handle_state_b`. The wheelchair observation includes relative chair position, relative chair velocity, chair forward direction, relative yaw rate, and centerline error. The handle observation includes handle positions in the robot-root frame and hand-to-handle position error.

The May 22 DirectObs branch now has a stronger observation set for the rail-free diagnostic: full wheelchair orientation basis, wheelchair linear/angular velocity, hand-to-handle relative orientation/angular velocity, and scaled incoming body wrench on the hands and handle bodies. That branch confirmed the wrench values are nonzero, but it still failed the Phase 1 stand gate, so the current blocker is not explained by missing chair velocity/orientation alone.

The hard-reference policy observes the robot state plus wheelchair-relative state, but it does not observe rail reaction force/torque. The completed 1 m/s full-resume task did not penalize rail yaw torque; `wheelchair_rail_yaw_torque` stayed at weight `0.0`. The failed 1 m/s yaw-torque task used a much stronger `-0.05` penalty and did not reproduce useful forward push behavior. The May 20 refinement branch tests a much smaller `-0.005` penalty from the good `model_19247.pt` actor.

The May 19 1 m/s run keeps the same observation shape, hard hand-handle attachment, forward-lean bias, reward weights, and PPO defaults as the reproducible 2 m/s path. The deliberate changes are the fixed command speed (`1.0 m/s`), velocity reward standard deviation (`0.4`), and progress cap (`1.4 m/s`) so the reward target is scaled to the lower speed without adding yaw-torque penalties.

## May 19 1 m/s Fast-Lean Hard Run

Task added in `unitree_rl_lab` commit `cf15f8b`:

`Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Velocity-Progress-Push-Attached-Hard`

The smoke run used `16` envs, loaded the same `model_13249.pt` source checkpoint, reset the critic, and set `--policy_std 0.02`. It completed one iteration with the intended reward stack: `wheelchair_track_forward_velocity = 10.0`, `wheelchair_forward_progress = 3.0`, `wheelchair_backward_velocity = -10.0`, `robot_forward_lean = 1.0`, and `wheelchair_rail_yaw_torque = 0.0`.

The long run is:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Velocity-Progress-Push-Attached-Hard \
  --max_iterations 5000 \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_x_rail_fast_lean_velocity_progress_push_attached/2026-05-18_00-37-49_minimal_x_rail_fast_2ms_forward_lean_rewardstd020_explorestd035_1024env_from_fixed_stand_12250/model_13249.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.02 \
  --run_name hard_attach_1mps_fastlean_long_from_13249_may19
```

Initial training trend around iteration `13252`: forward-progress reward rose to about `0.20`, `bad_orientation` stayed near zero, and `unstable_robot_state` was still zero. By the mid `13260`s, the same hard-attach guard pressure seen in the 2 m/s branch started appearing, so visual checkpoints are still required before calling the lower-speed branch better.

The run completed its requested 5000 additional iterations at `model_18248.pt`. The final scalar tail still showed weak train-time forward rewards, with `Episode_Reward/wheelchair_forward_progress` near `0.02`, `Episode_Termination/bad_orientation` around `0.26`, and `Episode_Termination/unstable_robot_state` around `0.68`. Deterministic playback remained useful enough to preserve the checkpoint for follow-up.

## May 19 Full Resume From `model_18248.pt`

A full optimizer-state resume was started from the completed 1 m/s checkpoint to test whether the policy can keep improving when continued directly from `model_18248.pt`. Unlike the original long 1 m/s run, this resume does not use `--load_model_only`, does not reset the critic, and does not override `policy_std`. It is therefore a true RSL-RL checkpoint continuation for actor, critic, optimizer, and exploration standard deviation.

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Velocity-Progress-Push-Attached-Hard \
  --max_iterations 1000 \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_03-18-18_hard_attach_1mps_fastlean_long_from_13249_may19/model_18248.pt \
  --run_name hard_attach_1mps_continue_full_from_18248_may19
```

The resume correctly continued at `Learning iteration 18251/19248`, confirming that `--max_iterations 1000` extended the target by 1000 iterations rather than restarting from zero. The first clean-reset rollouts were optimistic: `wheelchair_track_forward_velocity` briefly reached about `0.41`, `wheelchair_forward_progress` about `0.19`, and `unstable_robot_state` stayed near zero. After a few minutes, metrics returned toward the previous regime; by `model_18300.pt`, `wheelchair_forward_progress` was about `0.02` and `unstable_robot_state` was about `0.65`. At `model_18650.pt`, playback still showed useful forward walking/pushing behavior even though the train-time scalar tail remained noisy and low. The continuation finished its requested range at `model_19247.pt`, and the latest-video page was updated from that checkpoint on May 20.

Latest validation playback:

`logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-hard-attach-push-attached_model_19247_slow_revolve_best_20260520_012933/model_19247_slow_revolve_best.mp4`

Interpretation: the full resume works mechanically and preserved the useful forward-push behavior. Continue using videos, not scalar rewards alone, to decide whether the continuation is improving gait quality, because the train-time episode reward values are still dominated by reset distribution and instability terms.

There are now two resume modes worth keeping separate:

| Mode | Use | Command shape |
|---|---|---|
| Actor warm start | Start a new branch/task from a good actor while discarding old critic/optimizer state. This was used for the original 1 m/s speed-scaled run from the 2 m/s lineage. | `--resume --checkpoint <model.pt> --load_model_only --reset_critic --policy_std 0.02` |
| True RSL-RL continuation | Continue the exact same run lineage from a checkpoint, preserving actor, critic, optimizer, and exploration standard deviation. This was tested from `model_18248.pt` and still worked. | `--resume --checkpoint <model.pt>` |

The slow-revolve validation view can look like the video "splits" if it starts on env `0` and then switches to the best-moving env after a delay. The first `model_18650.pt` render used `follow_best_after_steps = 80`; at a 50 Hz sim step this is about `1.6 s`, matching the visible mid-clip jump. The corrected render used `--follow-best-after-steps 0`, so the best env is selected at frame zero. Treat the old delayed-switch clip as a camera-target artifact, not a policy discontinuity.

## May 20 Small Yaw-Torque Refinement

The first gait-refinement probe adds a small rail yaw-torque penalty without changing the successful straight-push reward shape. The goal is to discourage off-center twisting against the PhysX rail while preserving the forward push behavior from `model_19247.pt`.

Task added in `unitree_rl_lab` commit `cb89490`:

`Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Small-Yaw-Torque-Push-Attached-Hard`

Only intentional reward change from the completed 1 m/s full-resume task:

| Reward | Old weight | New weight |
|---|---:|---:|
| `wheelchair_rail_yaw_torque` | `0.0` | `-0.005` |

Training command:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Small-Yaw-Torque-Push-Attached-Hard \
  --max_iterations 500 \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_15-22-06_hard_attach_1mps_continue_full_from_18248_may19/model_19247.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.01 \
  --run_name small_yawtorque_0005_from_19247_may20
```

This is an actor warm start, not a true optimizer-state continuation, because the reward function changed. It should run from iteration `19248` through `19747`. The focused TensorBoard symlink `logs/tensorboard_focus/AAA_hard_attach_current` now points at this run.

Latest validation playback:

`logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-small-yaw-torque-hard-attach-push-attached_model_19300_slow_revolve_best_20260520_014829/model_19300_slow_revolve_best.mp4`

Early status: `unstable_wheelchair_state` remained near `0.0`, and the new yaw-torque penalty was present but very small. Train-time robot instability rose quickly, though: by about iteration `19327`, `unstable_robot_state` was back around `0.66` and `wheelchair_forward_progress` was only around `0.02`. The branch was stopped after the `model_19300.pt` playback render rather than spending the full 500-iteration budget. Treat this as a likely weak branch unless the deterministic playback shows an unexpectedly cleaner gait.

## May 20 Gait Sweep

The next refinement pass is a six-variant sweep from the same `model_19247.pt` parent. Each run uses actor warm start, reset critic, `--policy_std 0.01`, `2048` envs, and a short `60`-iteration budget. The goal is to generate comparable videos, not to declare a winner from TensorBoard alone.

Parent checkpoint:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_15-22-06_hard_attach_1mps_continue_full_from_18248_may19/model_19247.pt`

| Variant | Task | Intended change |
|---|---|---|
| `smooth_light` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Smooth-Light` | Initial smoothness test with small `action_rate`, `joint_acc`, and `energy` penalties. |
| `smooth_medium` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Smooth-Medium` | Stronger action-rate shaping with `joint_acc` disabled after the first branch showed large spikes. |
| `feet_light` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Feet-Light` | Light foot slide penalty plus small foot clearance reward. |
| `gait_light` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Gait-Light` | Light alternating gait reward plus foot slide/clearance shaping. |
| `posture_light` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Posture-Light` | Small base/waist/hip posture regularization while still allowing forward lean. |
| `reduced_scale` | `Unitree-G1-29dof-Wheelchair-GaitSweep-Reduced-Scale` | Smaller action scale for less aggressive stepping, plus tiny action-rate penalty. |

The latest-video page has been changed into a recent-video gallery so the sweep clips can be reviewed together:

`https://workstation.tailee9084.ts.net:8002/`

Completed sweep status, May 20:

All six branches trained for the intended short pass and produced `model_19250.pt`, `model_19300.pt`, and `model_19306.pt` checkpoints. The final checkpoints were not the best review targets: every branch had useful forward-progress and low-instability scalars near `model_19250.pt`, then collapsed by `model_19300.pt`/`model_19306.pt` into short episodes with weak forward progress and high `unstable_robot_state`. The comparison videos on the latest-video site therefore use `model_19250.pt` for each branch.

The first `smooth_light` run was started before the `joint_acc` term was patched out. Its train-time `joint_acc` reward showed very large spikes, so the smooth branches were changed to rely on action-rate and tiny energy terms instead of joint acceleration. `smooth_medium` and the later branches used the patched code.

| Variant | Run directory | Best review video |
|---|---|---|
| `smooth_light` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_smooth_light/2026-05-20_03-40-38_gait_sweep_smooth_light_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-smooth-light_model_19250_slow_revolve_best_20260520_053112/model_19250_slow_revolve_best.mp4` |
| `smooth_medium` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_smooth_medium/2026-05-20_03-48-10_gait_sweep_smooth_medium_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-smooth-medium_model_19250_slow_revolve_best_20260520_053249/model_19250_slow_revolve_best.mp4` |
| `feet_light` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_feet_light/2026-05-20_03-55-52_gait_sweep_feet_light_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-feet-light_model_19250_slow_revolve_best_20260520_053426/model_19250_slow_revolve_best.mp4` |
| `gait_light` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_gait_light/2026-05-20_04-39-42_gait_sweep_gait_light_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-gait-light_model_19250_slow_revolve_best_20260520_053604/model_19250_slow_revolve_best.mp4` |
| `posture_light` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_posture_light/2026-05-20_04-48-57_gait_sweep_posture_light_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-posture-light_model_19250_slow_revolve_best_20260520_053742/model_19250_slow_revolve_best.mp4` |
| `reduced_scale` | `logs/rsl_rl/unitree_g1_29dof_wheelchair_gait_sweep_reduced_scale/2026-05-20_04-58-15_gait_sweep_reduced_scale_from_19247_may20` | `logs/demos/unitree-wheelchair-gait-sweep-reduced-scale_model_19250_slow_revolve_best_20260520_053921/model_19250_slow_revolve_best.mp4` |

The quick scalar check that drove the video selection:

| Variant | `model_19250` forward progress / unstable robot | `model_19306` forward progress / unstable robot |
|---|---:|---:|
| `smooth_light` | `0.2080 / 0.0009` | `0.0280 / 0.6520` |
| `smooth_medium` | `0.2290 / 0.0008` | `0.0195 / 0.6517` |
| `feet_light` | `0.2213 / 0.0008` | `0.0254 / 0.6426` |
| `gait_light` | `0.2050 / 0.0010` | `0.0231 / 0.6614` |
| `posture_light` | `0.2255 / 0.0005` | `0.0224 / 0.6536` |
| `reduced_scale` | `0.2097 / 0.0001` | `0.0249 / 0.6174` |

Takeaway: the immediate warm-start behavior still looks useful, but the 60-iteration PPO continuations are too aggressive for these small shaping changes. Next refinement should test a much smaller learning rate, fewer update epochs, or a shorter checkpoint-selection cadence before adding stronger gait shaping.

## May 20 Continuation Diagnosis

The sweep result raised a basic question: was the gait shaping bad, or was continuing PPO from `model_19247.pt` bad even without reward changes? Three control runs were executed from the same parent checkpoint:

| Test | Task/setup | Run directory |
|---|---|---|
| `baseline_same_task` | Same 1 m/s hard-attach task, actor-only warm start, reset critic, `--policy_std 0.01`. | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-20_06-36-08_baseline_same_task_from_19247_may20` |
| `true_resume_same_task` | Same task with full checkpoint resume: actor, critic, optimizer, and checkpoint policy std loaded. | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-20_06-44-59_true_resume_same_task_from_19247_may20` |
| `conservative_ppo` | Same task/rewards, but new conservative PPO runner with `learning_rate=1e-5`, `clip_param=0.03`, `num_learning_epochs=1`, `entropy_coef=0.0`, `desired_kl=0.002`, `max_grad_norm=0.5`; launched actor-only with reset critic, `--policy_std 0.005`, and `--freeze_policy_std`. | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_conservative_ppo_hard_attach_push_attached/2026-05-20_06-53-39_conservative_ppo_from_19247_may20` |

The control metrics matched the sweep failure mode. All three were still good around `model_19250.pt`, then collapsed by the final `model_19306.pt`.

| Test | `model_19250` forward progress / unstable robot | `model_19306` forward progress / unstable robot |
|---|---:|---:|
| `baseline_same_task` | `0.2150 / 0.0004` | `0.0229 / 0.6387` |
| `true_resume_same_task` | `0.1787 / 0.0016` | `0.0209 / 0.6539` |
| `conservative_ppo` | `0.2115 / 0.0007` | `0.0169 / 0.6520` |

Control videos on the latest-video gallery:

| Test | Early checkpoint | Final checkpoint |
|---|---|---|
| `baseline_same_task` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-hard-attach-push-attached_model_19250_slow_revolve_best_20260520_070219/model_19250_slow_revolve_best.mp4` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-hard-attach-push-attached_model_19306_slow_revolve_best_20260520_070355/model_19306_slow_revolve_best.mp4` |
| `true_resume_same_task` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-hard-attach-push-attached_model_19250_slow_revolve_best_20260520_070531/model_19250_slow_revolve_best.mp4` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-hard-attach-push-attached_model_19306_slow_revolve_best_20260520_070707/model_19306_slow_revolve_best.mp4` |
| `conservative_ppo` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-conservative-ppo-hard-attach-push-attached_model_19250_slow_revolve_best_20260520_070843/model_19250_slow_revolve_best.mp4` | `logs/demos/unitree-wheelchair-physx-rail-1mps-fast-lean-conservative-ppo-hard-attach-push-attached_model_19306_slow_revolve_best_20260520_071019/model_19306_slow_revolve_best.mp4` |

Conclusion: the collapse is not caused by the gait-sweep reward terms, and it is not fixed by true-resuming the optimizer state or by the first conservative PPO runner. The current best interpretation is that `model_19247.pt` is a useful playback/checkpoint-selection point but is fragile under more PPO updates on this rollout distribution. The next serious refinement should either continue from an earlier reproducible checkpoint path before the fragility appears, or switch to a guarded fine-tuning method that accepts/rejects checkpoints frequently instead of assuming a 60-iteration continuation is safe.

## May 19 Rollback

The 1 m/s yaw-torque hard branch was stopped after the `model_15000.pt` playback showed poor behavior. Compared with the 2 m/s fast-lean reference, that branch lowered the command from `2.0 m/s` to `1.0 m/s`, removed the forward-lean reward, softened the backward-velocity penalty from `-10.0` to `-3.0`, added `wheelchair_rail_yaw_torque = -0.05`, and reset the critic from the `model_13300.pt` actor. In practice it produced short bad episodes with high `unstable_robot_state` and little useful forward progress.

Training was rolled back to the 2 m/s fast-lean hard task from the preserved `model_13300.pt` checkpoint. The latest-video site was also restored to that 2 m/s reference playback before the conservative continuation test, so it is not showing the failed 1 m/s branch.

The immediate PPO restart from `model_13300.pt` was stopped after a few minutes because it did not follow the same trajectory as the original uninterrupted run: episode length dropped sharply, `unstable_robot_state` rose above `0.6`, and forward reward stayed weak. Treat `model_13300.pt` as the known-good reference checkpoint, but do not treat a fresh process launched from that checkpoint as an exact continuation of the original training path.

A conservative continuation was started from `model_13300.pt` on the same 2 m/s fast-lean hard task using `--policy_std 0.005 --freeze_policy_std`. This kept the reward/task shape from the good run but reduced exploration noise so the policy would drift less aggressively. It was stopped at the first new checkpoint, `model_13350.pt`, because the metrics still degraded: by about iteration `13341`, `unstable_robot_state` was about `0.65`, `bad_orientation` was about `0.26`, and forward rewards were weak.

The stronger May 19 check was to reproduce the original path from the earlier X-rail fast-lean checkpoint rather than continuing from `model_13300.pt`. Two short runs used the original command shape:

```bash
python scripts/rsl_rl/train.py \
  --headless \
  --num_envs 2048 \
  --task Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-Fast-Lean-Velocity-Progress-Push-Attached-Hard \
  --resume \
  --checkpoint logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_x_rail_fast_lean_velocity_progress_push_attached/2026-05-18_00-37-49_minimal_x_rail_fast_2ms_forward_lean_rewardstd020_explorestd035_1024env_from_fixed_stand_12250/model_13249.pt \
  --load_model_only \
  --reset_critic \
  --policy_std 0.02
```

`2026-05-19_02-08-33_hard_attach_repro_13300_from_13249_may19/model_13300.pt` hashes exactly the same as the May 18 `model_13300.pt`: `46921180444f62957a9236f84654adad5b10ec0583c60883d1f0b4cecefeb248`. `2026-05-19_02-22-02_hard_attach_repro_13350_from_13249_may19/model_13350.pt` also hashes exactly the same as the May 18 `model_13350.pt`: `68636d72d464fb2d4ab8b78797363a878eb6753e46b57889d5ca54445663f819`.

Practical takeaway: we have not lost the original good training path. If the goal is to train "like before," start again from the `model_13249.pt` source with the old command shape and let it run uninterrupted past the known checkpoints. Restarting from `model_13300.pt` is still useful for experiments, but it does not preserve the same rollout/environment state as the uninterrupted run and should not be treated as an exact continuation of that path.

## May 19 Git-History Audit

The hard-attach history matters because the good visual behavior was not imaginary. The relevant commits were:

| Commit | Time | Change |
|---|---|---|
| `e6b04c9` | May 18, 17:37 | Added the hard-attach wheelchair rail task. |
| `c267f5a` | May 18, 18:12 | Fixed hard wheelchair attachment startup. |
| `08916ba` | May 18, 18:53 | Fixed hard attach USD joint-frame alignment. |
| `53da64e` | May 18, 19:34 | Disabled PhysX rail command debug markers. |
| `4500d7d` | May 18, 19:41 | Added observation clips and `unstable_*_state` runaway terminations. |
| `9a9c2d4` | May 18, 19:47 | Loosened those runaway thresholds. |

The good-looking run was started before `4500d7d`:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_fast_lean_hard_attach_push_attached/2026-05-18_19-34-44_hard_attach_clean_2048_from_13249`

That pre-guard run reached early `Episode_Reward/wheelchair_forward_progress` values around `0.20` to `0.26`, with useful-looking chair motion. After `4500d7d`, forward progress collapsed into the `0.02` to `0.04` range while `Episode_Termination/unstable_robot_state` climbed rapidly. Loosening the thresholds in `9a9c2d4` reduced the immediate reset pressure but did not restore sustained learning.

The conclusion is not "bad checkpoint." The conclusion is that the hard hand-handle joint setup can produce catastrophic PhysX outliers. The low runaway guard hid the crash by resetting those states, but it also reset enough of the rollout to destroy the forward-push training signal.

## May 19 Hard-Attach Diagnostics

Several diagnostic task variants were added to isolate the failure:

| Task suffix | Purpose | Result |
|---|---|---|
| `Hard-NoGuard` | Match the pre-runaway-guard behavior by removing clips and unstable-state terminations. | Recovered good early forward-progress metrics, then crashed with invalid policy std after huge velocity/value spikes. |
| `Hard-NoTerminate` | Keep observation clips but remove unstable-state early termination. | Still recovered forward progress, but PPO still blew up from extreme finite states. |
| `Hard-Robust` | Keep no broad runaway termination, add non-finite resets, cap optional chair penalties, use fixed low-LR PPO with `value_loss_coef=0.0`. | 2048 envs hit PhysX GPU articulation kernel failures; 256 envs completed the short run but still produced extreme finite velocity metrics and later non-finite terminations. |

The 2048-env robust reset-optimizer test failed below PPO with PhysX CUDA errors such as `GPU artiPropagateVelocity fail to launch kernel`, `PhysX Internal CUDA error`, and `Failed to get DOF velocities from backend`. The 256-env version avoided the immediate CUDA crash, but by the later iterations it logged huge finite velocity metrics, non-finite robot/chair terminations around `0.0117`, and poor forward reward. Its saved `model_13324.pt` is a diagnostic artifact only and should not be used as a new warm-start.

Current diagnosis: both-hand hard spherical joints across the robot and wheelchair can work visually for playback and short windows, but at training scale they create rare unstable constraint states. At 2048 envs those states can break the GPU PhysX articulation solver; at lower env counts they still poison rollouts unless caught. This is why the old checkpoint looked good, why the broad guard made training look bad, and why simply continuing the same task is unreliable.

Next useful experiments should change the physics interface, not just PPO knobs. Options are a catastrophic-only finite guard with much higher thresholds than `4500d7d`, fewer envs while debugging, a better hand-handle constraint that avoids a stiff closed-loop across two articulations, or a return to a compliant attachment where the policy observes the attachment load. Do not train overnight from the current hard-attach robust diagnostic checkpoints.

## Current Issue

The hard hand-handle joint version is still the best visual reference. The warning root cause is now understood: the imported G1 USD stage has the arms in its authored/default pose, while the reset event moves the arms into the wheelchair-handle pose before the hard hand-handle joints are created. Runtime palm-grip alignment was already good at about `5e-6 m`, but the authored USD joint frames were about `0.128 m` apart, so PhysX reported the joints as disjoint and warned that the bodies may snap together.

`events.py` now mirrors the reset runtime poses for the relevant hand and handle bodies into the USD stage before creating those cross-asset joints, preserves existing USD xform precision, authors the local joint frame before binding the body targets, and only performs the stage sync for envs where the joints still need to be created.

Final verification from `model_13249.pt` on `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-Fast-Lean-Velocity-Progress-Push-Attached-Hard`:

| Check | Result |
|---|---:|
| Runtime palm-grip error | about `5e-6 m` |
| USD stage joint-frame error | about `5e-6 m` |
| `CreateJoint - found a joint with disjointed body transforms` | not present |
| 16-env one-iteration training smoke | completed |

Training smoke log:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_fast_lean_hard_attach_push_attached/2026-05-18_18-52-06_hard_stage_runtime_sync_final_1iter_smoke`

Remaining warning audit:

| Message | Status |
|---|---|
| `CreateJoint - found a joint with disjointed body transforms` | Fixed by syncing the hand/handle USD stage xforms to the reset runtime pose before authoring the hard joints. |
| `FabricManager::initializePointInstancer mismatched prototypes` | Removed for the PhysX rail push tasks by disabling command debug markers in headless training. |
| `Unresolved reference prim path ... /visuals/<link>` from the wheelchair USD | Known URDF-import visual-reference noise. The X-rail wheelchair URDF only has a real detailed visual on `base_link`; the importer still authors visual references for no-visual helper links such as rail, casters, and handle frames. Smoke training verifies this is not blocking physics. A clean long-term fix is a pre-converted/patched training USD or placeholder visuals on those helper links. |
| `Not all actuators are configured! 0 != 7` for the wheelchair | Expected for the passive wheelchair articulation. The chair joints are not actuated by the policy. |

One real training issue remained after the joint fix: a few hard-attach envs could produce enormous finite robot velocities, which poisoned the critic even though the values were not NaN/Inf. Commit `4500d7d` added task-local unstable-state terminations and observation clips for the PhysX rail branch. The first guard thresholds were too tight and reset too much of the rollout, so commit `9a9c2d4` loosened them to catch only catastrophic runaway states. The 2048-env loose-guard smoke no longer showed the earlier huge velocity metric or exploding critic loss; the live run now logs `Episode_Termination/unstable_robot_state`, which should be monitored alongside the forward rewards.

The hard-reference playback is visually useful, but speed alone is not a success metric. The latest diagnostic for `model_13249.pt` on the hard fast-lean task reported a `2.0 m/s` command, `1.1268 m/s` mean wheelchair forward speed, and `0.000` of samples within `0.10 m/s` of the command. Treat that as chair-motion telemetry, not proof of command tracking or gait quality.

## May 18 Soft-Observation Branch

Commit `8037cc8` adds two things:

| Change | Purpose |
|---|---|
| `--print-soft-attachment-stats` in `scripts/rsl_rl/play.py` | Playback-only diagnostic for hand-handle spring position error, relative velocity, force, force imbalance, and axis alignment. |
| `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Yaw-Torque-Push-Attached-SoftObs` | New task ID that exposes soft-attachment state to the policy and critic. |

The new observation term is `wheelchair_soft_attachment_state`. It adds relative hand-handle velocity, handle axes in the hand frames, capped spring force, and spring force norm. Because observations use history stacking, this increases policy observation shape from `(585,)` to `(745,)` and critic observation shape from `(600,)` to `(760,)`.

The baseline checkpoint was expanded with zero-initialized input weights:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_softobs_push_attached/from_physx_yawtorque_model_15900_softobs/model_15900.pt`

Baseline deterministic playback from the original `model_15900.pt` over `300` steps and `10` envs:

| Metric | Value |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.0007 m/s` |
| Within `0.10 m/s` of command | `0.000` |
| Rail yaw torque abs mean | `49.38 Nm` |
| Rail yaw torque abs p95 | `72.92 Nm` |
| Soft attachment position error mean | `0.0423 m` |
| Soft attachment position error p95 | `0.0456 m` |
| Soft attachment force norm mean | `103.75 N` |
| Soft attachment force imbalance mean | `45.24 N` |

Expanded SoftObs playback before training still matched the old actor behavior, as expected: the added observation weights start at zero, so the actor cannot use the new signal until PPO updates it.

The first `4096`-env smoke run trained from `15900` to `15949` with no base-height, wheelchair non-finite, or robot non-finite terminations in the logged iterations. Train-time forward rewards rose slightly, but deterministic playback still showed almost no useful chair movement:

| Metric | `model_15949.pt` |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.0016 m/s` |
| Within `0.10 m/s` of command | `0.000` |
| Rail yaw torque abs mean | `46.67 Nm` |
| Rail yaw torque abs p95 | `69.85 Nm` |
| Soft attachment position error mean | `0.0428 m` |
| Soft attachment position error p95 | `0.0532 m` |
| Soft attachment relative velocity mean | `0.1193 m/s` |
| Soft attachment force norm mean | `106.87 N` |
| Soft attachment force imbalance mean | `50.95 N` |

This is a useful smoke result, not a success result. It suggests the added observations did not destabilize the run, but `50` iterations were not enough to turn the hidden load signal into a working push. The active continuation is:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_softobs_push_attached/2026-05-18_16-31-49_softobs_4096_250_from_15949`

That continuation started at iteration `15949` and saved `model_16198.pt`. Rewards were kept unchanged during this gate so the comparison stayed clean.

Interim trend at about iteration `16044`: the run is still numerically stable and non-finite terminations are `0.0`, but the chair-forward signal is weak. Recent `wheelchair_track_forward_velocity` values are mostly around `0.02` to `0.04`, `wheelchair_forward_progress` is still only a few thousandths to about `0.01`, and backward-velocity penalty is starting to appear. Let the gate finish, but do not treat the current trend as evidence that SoftObs has solved the forward-push problem.

Interim trend at about iteration `16109`: the run has started showing intermittent forward signal. Some recent iterations reached `wheelchair_track_forward_velocity` around `0.10` to `0.12` and `wheelchair_forward_progress` around `0.03`, but the signal is noisy. Backward-velocity penalty and base-height terminations are also rising, so this may be unstable/exploratory motion rather than a reliable push. The final deterministic playback is required before deciding whether to extend this branch.

Final deterministic playback from `model_16198.pt` did not validate the train-time spikes:

| Metric | `model_16198.pt` |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.0004 m/s` |
| Forward max | `0.0982 m/s` |
| Within `0.10 m/s` of command | `0.000` |
| Rail yaw torque abs mean | `47.59 Nm` |
| Rail yaw torque abs p95 | `73.24 Nm` |
| Soft attachment position error mean | `0.0416 m` |
| Soft attachment relative velocity mean | `0.0484 m/s` |
| Soft attachment force norm mean | `102.29 N` |
| Soft attachment force imbalance mean | `37.02 N` |

Conclusion: adding soft-attachment observations improved some attachment diagnostics, but it did not produce a reliable deterministic forward push. The train-time reward spikes were likely exploratory/stochastic behavior rather than a usable policy. Do not extend this exact branch without changing the attachment model or the state/action structure.

## May 18 Stiff Soft-Observation Branch

Commit `d1c0d47` adds `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Yaw-Torque-Push-Attached-SoftObs-Stiff`.

This branch keeps the same reward terms and same `(745,)` policy observation shape as SoftObs, but changes the bounded hand-handle spring-damper:

| Parameter | SoftObs | SoftObs-Stiff |
|---|---:|---:|
| Stiffness | `2500` | `5000` |
| Damping | `75` | `150` |
| Max force | `350 N` | `500 N` |
| Observation force scale | `350 N` | `500 N` |

Baseline deterministic playback from the same expanded `model_15900.pt` actor was stable but still stationary:

| Metric | SoftObs-Stiff baseline |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.0003 m/s` |
| Forward max | `0.1229 m/s` |
| Within `0.10 m/s` of command | `0.000` |
| Rail yaw torque abs mean | `48.48 Nm` |
| Soft attachment position error mean | `0.0411 m` |
| Soft attachment force norm mean | `202.17 N` |
| Soft attachment force imbalance mean | `52.91 N` |

Active smoke run:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_softobs_stiff_push_attached/2026-05-18_17-07-46_stiff_softobs_4096_smoke_from_15900`

This run starts from the expanded `model_15900.pt` actor, resets the critic, sets policy std to `0.015`, uses `4096` envs, and targets a `50`-iteration smoke gate before any longer continuation.

Final deterministic playback from the stiff smoke `model_15949.pt` did not improve the result:

| Metric | SoftObs-Stiff `model_15949.pt` |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.0003 m/s` |
| Forward max | `0.2011 m/s` |
| Within `0.10 m/s` of command | `0.000` |
| Rail yaw torque abs mean | `50.99 Nm` |
| Rail yaw torque abs p95 | `79.81 Nm` |
| Rail yaw torque abs max | `986.59 Nm` |
| Soft attachment position error mean | `0.0468 m` |
| Soft attachment relative velocity mean | `0.7238 m/s` |
| Soft attachment force norm mean | `217.55 N` |
| Soft attachment force max | `500.00 N` |
| Soft attachment force imbalance mean | `99.30 N` |

Conclusion: the stiffer bounded spring made the attachment more violent without producing deterministic forward motion. It hit the force cap and worsened yaw/imbalance metrics. Do not extend this branch as-is.

## May 18 Hard-Attach Retest

Task added:

`Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Yaw-Torque-Push-Attached-Hard`

This variant returned to both hand-handle hard USD joints on the 1 m/s yaw-torque PhysX rail setup. It disabled the soft hand-handle spring events and restored `attach_wheelchair_hands_to_handles` with both `left_rubber_hand -> left_handle_frame` and `right_rubber_hand -> right_handle_frame` spherical joints. The observation shape stayed compatible with the old non-SoftObs actor: policy `(585,)`, critic `(600,)`.

Baseline deterministic playback used:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_push_attached/2026-05-18_14-15-04_soft_attach_overnight_12288env_from_15400_after_video/model_15900.pt`

Before the stage-pose sync fix, playback emitted:

`CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together`

That warning should not be interpreted as proof that the palm grip was centimeters away from the handle at runtime. The measured runtime grip anchor was already close; the body origins are offset by design. The actual mismatch was between the reset runtime pose and the authored USD stage pose. After syncing the relevant body xforms to the reset pose before joint creation, the hard fast-lean startup diagnostic and a 16-env training smoke produced no `CreateJoint` disjoint-body warning.

The old `model_15900.pt` deterministic playback moved the chair but did not show a good walking push:

| Metric | Hard attach playback |
|---|---:|
| Commanded wheelchair X velocity | `1.0000 m/s` |
| Measured forward mean | `0.6810 m/s` |
| Forward min | `-0.1955 m/s` |
| Forward max | `1.2117 m/s` |
| Within `0.10 m/s` of command | `0.295` |
| Rail yaw torque abs mean | `83.06 Nm` |
| Rail yaw torque abs p95 | `212.25 Nm` |
| Rail yaw torque abs max | `1108.07 Nm` |

A slow-orbit playback video was published to the latest-video site using the new `isaac-clip` project preset:

`unitree-wheelchair-minimal-physx-rail-1mps-yaw-torque-hard-attach-push-attached`

A second site render used `--video-follow-best-robot` to rule out a bad fixed env-0 camera choice. The visual result still is not a good walking push. The chair moves in the rollout, but the robot posture/gait is poor and the motion looks like hard-joint forcing rather than a learned stable forward walk. Treat the speed table above as a coupling diagnostic, not a success metric.

Training smoke:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_hard_attach_push_attached/2026-05-18_17-28-47_hardattach_spherical_1024_smoke_from_15900`

This smoke used `1024` envs, loaded the old actor from `model_15900.pt`, reset the critic, and set policy std to `0.015`. It started training and showed useful forward reward, but it ended at iteration `15921` without saving a new checkpoint. TensorBoard event summary:

| Scalar | First logged | Last logged |
|---|---:|---:|
| `Train/mean_reward` | `0.14` at `15900` | `12.70` at `15921` |
| `Episode_Reward/wheelchair_track_forward_velocity` | `0.0051` | `0.2901` |
| `Episode_Reward/wheelchair_forward_progress` | `0.0054` | `0.1151` |
| `Episode_Reward/wheelchair_backward_velocity` | `-0.0001` | `-0.0147` |
| `Episode_Reward/wheelchair_rail_yaw_torque` | `-0.0009` | `-0.0068` |
| `Episode_Termination/non_finite_wheelchair` | `0.0000` | `0.0040` |
| `Episode_Termination/non_finite_robot` | `0.0000` | `0.0050` |

Conclusion: this `model_15900.pt` retest was the wrong comparison point for the visually good behavior. The branch to preserve is the older fast-lean hard-reference checkpoint, `model_13249.pt`, not the later soft-adapted/yaw-torque lineage.

## Run Lineage

| Family | Result |
|---|---|
| Kinematic visual demo, May 15, 2026 | Good-looking demo only; no physical chair dynamics. |
| Handle-grip walking, May 15, 2026 | Taught the walking policy to keep hands near handle targets. |
| Dynamic free-chair attempts, May 15, 2026 | Added passive wheelchair dynamics, contact rewards, and invalid-contact penalties; tended to veer or collapse. |
| Standing bridge runs, May 16, 2026 | Tried to make the robot stand with the chair/handles before walking; several startup/ragdoll diagnostics were captured. |
| Minimal X-rail runs, May 17, 2026 | Simplified the problem to forward/back chair motion; exposed backward-walking and off-center pushing exploits. |
| PhysX rail diagnostic, May 18, 2026 | Replaced kinematic rail clamp with real prismatic articulation so yaw reaction torque could be measured. |
| PhysX rail soft-attachment run, May 18, 2026 | Stable large-env setup, but deterministic playback remained stationary. |
| PhysX rail SoftObs and SoftObs-Stiff, May 18, 2026 | Exposed attachment/load state and tested a stiffer bounded spring. Neither produced reliable deterministic forward motion; the stiff version worsened force/yaw spikes. |
| PhysX rail hard-attach retest, May 18, 2026 | Both hand-handle hard joints moved the chair in playback. The joint-snap warning was fixed, but later rail/free-chair transfer still did not solve the rail-free task. |
| Rewritten scratch damping curriculum, May 23, 2026 | Active path. Train from zero through Phase 1A-1E, gradually reducing chair planar damping before adding any forward-push objective. |

Detailed run commands, old checkpoints, asset turntables, and startup/ragdoll videos are kept in the [chronological archive](archive.md).

## Metrics To Watch

Use the focused TensorBoard on port `6007` for the current run. During rewritten Phase 1, prioritize stability and chair-drift metrics over forward-speed metrics.

| TensorBoard scalar | Meaning |
|---|---|
| `Episode_Termination/bad_orientation` | Robot base/torso tipped past the orientation limit. This should fall as standing is learned. |
| `Episode_Termination/base_height` | Robot falling or collapsing low. |
| `Episode_Termination/time_out` | Healthy full-length episodes. Higher is better for Phase 1. |
| `Episode_Reward/wheelchair_root_position` | Chair root drift penalty. More negative means the chair is drifting from reset. |
| `Episode_Reward/wheelchair_yaw_velocity` | Chair twist-rate penalty. More negative means more yaw motion. |
| `Episode_Reward/wheelchair_lateral_velocity` | Side-motion penalty. More negative means more lateral motion. |
| `Episode_Reward/robot_hand_wrench` | Load through the robot hand bodies. Closer to `0` means less leaning/pulling through the attachment. |
| `Episode_Reward/wheelchair_handle_wrench` | Load at the wheelchair handle bodies. Closer to `0` means less handle abuse. |
| `Episode_Reward/wheelchair_track_forward_velocity` | Should stay `0.0` during Phase 1. If nonzero, the stand/hold stage accidentally has a push-speed reward. |
| `Episode_Reward/wheelchair_forward_progress` | Should stay `0.0` during Phase 1. This becomes useful only after forward-push phases start. |

Do not treat a single train-time reward scalar as proof of success. For Phase 1, the useful validation is deterministic playback from a fixed reset: the robot should stand with hands attached, the chair should stay settled, and the startup should not show a delayed snap or collapse.

## Next Fixes To Discuss

The current evidence says the direct release was too abrupt, not just under-observed. The immediate plan is to finish Phase 1A, render it, and only then decide whether to continue to Phase 1B or adjust the damping/reward weights.

Current decision points:

1. If Phase 1A still has high `bad_orientation` after a few hundred iterations, reduce early action scale or make the first gate more like a pure standing task.
2. If Phase 1A stands but leans hard through the handles, increase `robot_hand_wrench` and `wheelchair_handle_wrench` penalties before moving to Phase 1B.
3. If Phase 1B fails immediately, add an intermediate damping stage between `0.0/0.0/0.0` and `0.05/0.02/0.02`.
4. Do not add forward progress, velocity tracking, or turning until Phase 1E can stand/hold without artificial planar damping.

The old SoftObs, fixed-chair, and direct rail-free release branches are archived diagnostics. Do not keep extending them unless there is a specific comparison question.
