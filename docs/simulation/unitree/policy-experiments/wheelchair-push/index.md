# G1 Wheelchair Push Policy

This page is the working summary for the wheelchair-push policy experiments. Keep current decisions here and move detailed chronological notes to the [archive](archive.md).

## Current Status

| Item | Value |
|---|---|
| Active task | `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Yaw-Torque-Push-Attached-Hard` |
| Main config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Observation helpers | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py` |
| Attachment helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/events.py` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_hard_attach_push_attached/` |
| Current run | `2026-05-18_17-28-47_hardattach_spherical_1024_smoke_from_15900` |
| Summary last updated | May 18, 2026, 17:55 Toronto |
| Training tmux | none, hard-attach smoke ended |
| Training env count | `1024` smoke |
| Latest-video page | `https://workstation.tailee9084.ts.net:8002/` |
| Focused TensorBoard | `http://workstation.tailee9084.ts.net:6007/` |

The previous soft-attachment run was stopped at `model_15900.pt` and used as the baseline for the SoftObs branch.

The first SoftObs smoke run completed cleanly at `model_15949.pt`. It is safe enough to continue, but deterministic playback still does not show meaningful chair motion: measured forward speed was `0.0016 m/s` against the `1.0 m/s` command. A `250`-iteration continuation is now running from that checkpoint.

## Current Task Shape

The current wheelchair is not free in all directions. It uses the PhysX rail URDF:

`assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair_x_rail.urdf`

That asset fixes `rail_world` and connects the moving `base_link` through a prismatic `rail_x_joint`. The chair can move forward/back along X; yaw, lateral motion, roll, and pitch are constrained by physics instead of the older kinematic root-pose clamp.

The active nonzero reward terms are intentionally small:

| Reward | Weight | Purpose |
|---|---:|---|
| `wheelchair_track_forward_velocity` | `10.0` | Match wheelchair `base_link` forward velocity to the fixed `1.0 m/s` command. |
| `wheelchair_forward_progress` | `2.0` | Reward positive world-X wheelchair movement. |
| `wheelchair_backward_velocity` | `-3.0` | Penalize moving the chair backward. |
| `wheelchair_rail_yaw_torque` | `-0.05` | Penalize twisting the constrained rail/chair instead of pushing straight. |

Inherited locomotion, pose, contact, hand-position, and wrist terms are currently set to `0.0` for this task unless explicitly listed above.

## Observation Gap

The policy observes the robot state plus wheelchair-relative state through `wheelchair_root_state_b` and `wheelchair_handle_state_b`. The wheelchair observation includes relative chair position, relative chair velocity, chair forward direction, relative yaw rate, and centerline error. The handle observation includes handle positions in the robot-root frame and hand-to-handle position error.

The policy does not currently observe the full soft-attachment state. Missing pieces include relative hand-handle orientation, relative hand-handle velocity, spring force, attachment force balance between left/right hands, and rail reaction force/torque as an observation. This matters because the current physical attachment is compliant.

## Current Issue

The hard hand-handle joint version looked better in short visual playback, but it produces PhysX joint snap warnings because the hand bodies and handle bodies start apart and are snapped together by the joint. The bounded spring-damper attachment fixed that specific warning and allowed high-env probes up to `12288` environments, but it did not learn a deterministic forward push.

The current evidence favors the hard attachment for learnability and the spring-damper for numerical cleanliness. The spring-damper introduces small hidden motions and loads between the rubber hands and handles. From the policy's point of view, two states can look almost identical in observation space while the chair reacts differently because the spring load, relative handle orientation, or contact mode is different. The useful names for this failure mode are `contact-rich hybrid dynamics`, `stiff non-smooth dynamics`, `partial observability`, and `state aliasing`.

That makes reward-only tuning unreliable. PPO can see low or noisy returns, but it cannot cleanly assign credit if the important attachment/load state is not observable. The current low `wheelchair_forward_progress` and low `wheelchair_track_forward_velocity` after the soft-attachment restart are consistent with this issue.

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

This variant returns to both hand-handle hard USD joints on the current PhysX rail setup. It disables the soft hand-handle spring events and restores `attach_wheelchair_hands_to_handles` with both `left_rubber_hand -> left_handle_frame` and `right_rubber_hand -> right_handle_frame` spherical joints. The observation shape stays compatible with the old non-SoftObs actor: policy `(585,)`, critic `(600,)`.

Baseline deterministic playback used:

`logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_push_attached/2026-05-18_14-15-04_soft_attach_overnight_12288env_from_15400_after_video/model_15900.pt`

Playback still emits the expected PhysX warning:

`CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together`

Despite that, deterministic playback did work and moved the chair:

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

Conclusion: the both-hand hard attachment is worth revisiting as a coupling mechanism, but the current playback is not a successful walking policy. It is not physically clean, the reaction torque is high, and the useful forward speed appears partly driven by hard-joint coupling/snap rather than clean locomotion. The next version should try to keep the hard coupling learnability while reducing the snap/stress at reset, then require visual gait validation before calling it progress.

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
| PhysX rail hard-attach retest, May 18, 2026 | Both hand-handle hard joints moved the chair in playback despite joint-snap warnings, but the visual gait was bad; 1024-env smoke started learning but ended before checkpointing. |

Detailed run commands, old checkpoints, asset turntables, and startup/ragdoll videos are kept in the [chronological archive](archive.md).

## Metrics To Watch

Use the focused TensorBoard on port `6007` for the current comparison set.

| TensorBoard scalar | Meaning |
|---|---|
| `Episode_Reward/wheelchair_track_forward_velocity` | Main 1 m/s chair velocity tracking reward. Low means the chair is not matching the target speed. |
| `Episode_Reward/wheelchair_forward_progress` | Positive forward chair movement. Low means the chair barely moves forward. |
| `Episode_Reward/wheelchair_backward_velocity` | Negative penalty for backward chair movement. More negative is worse. |
| `Episode_Reward/wheelchair_rail_yaw_torque` | Penalty for twisting the rail/chair instead of pushing straight. More negative means more off-axis abuse. |
| `Episode_Termination/base_height` | Robot falling or collapsing low. |
| `Episode_Termination/non_finite_wheelchair` | Physics failure in the wheelchair articulation. Should stay `0.0`. |
| `Episode_Termination/non_finite_robot` | Physics failure in the robot articulation. Should stay `0.0`. |

Do not treat a single train-time reward scalar as proof of success. For this task, the useful validation is deterministic playback from a fixed reset with measured wheelchair forward velocity and rail reaction torque.

## Next Fixes To Discuss

The current evidence says this is not just a reward-weight problem. The policy can keep the attachment numerically stable, but it is not discovering a repeatable way to convert arm motion into forward rail motion.

Best next structural tests:

1. Fix the hard-attach reset geometry so hands and handles start coincident before the joints are created. This directly targets the PhysX snap warning without losing hard coupling.
2. Try hard attach at lower env count first, then scale up only after non-finite terminations stay near zero.
3. Add direct rail-force / rail-joint observation and TensorBoard metrics. We already print rail wrench in playback, but the policy does not observe that reaction load during training.
4. Add a capped yaw/side-load penalty only after the hard attachment can train long enough to checkpoint. The current hard playback has real forward motion but high rail reaction torque.
5. If both-hand hard attachment keeps producing bad simulator states, try a one-hand rigid or spherical probe to remove the two-arm closed loop while preserving hard force transfer.

Do not keep running the plain SoftObs or SoftObs-Stiff branches without changing the task structure; both failed deterministic validation.
