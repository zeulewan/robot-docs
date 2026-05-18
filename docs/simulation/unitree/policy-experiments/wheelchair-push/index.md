# G1 Wheelchair Push Policy

This page is the working summary for the wheelchair-push policy experiments. Keep current decisions here and move detailed chronological notes to the [archive](archive.md).

## Current Status

| Item | Value |
|---|---|
| Active task | `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Yaw-Torque-Push-Attached` |
| Main config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Observation helpers | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py` |
| Soft attachment helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/events.py` |
| Experiment root | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_push_attached/` |
| Current run | `2026-05-18_14-15-04_soft_attach_overnight_12288env_from_15400_after_video` |
| Summary last updated | May 18, 2026, 16:20 Toronto |
| Training tmux | `unitree_wheelchair_physx_12288_train` |
| Training env count | `12288` |
| Latest-video page | `https://workstation.tailee9084.ts.net:8002/` |
| Focused TensorBoard | `http://workstation.tailee9084.ts.net:6007/` |

The current run is numerically stable, but the behavior is not solved. Around iteration `15681`, training was still producing weak chair-forward terms: `wheelchair_track_forward_velocity` about `0.084`, `wheelchair_forward_progress` about `0.035`, `wheelchair_backward_velocity` about `-0.089`, and `wheelchair_rail_yaw_torque` about `-0.020`. That means the policy is staying mostly stable, but it is not yet producing a clean forward push.

The previous soft-attachment run was stopped at `model_15900.pt` and used as the baseline for the next experiment branch.

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

The hard hand-handle joint version looked better in short visual playback, but it was not stable enough for large parallel training. It produced PhysX joint snap warnings, value-loss spikes, and occasional bad simulator states. The bounded spring-damper attachment fixed that stability problem: high-env probes reached `12288` environments cleanly, and `CreateJoint` warnings disappeared.

The tradeoff is learnability. The spring-damper introduces small hidden motions and loads between the rubber hands and handles. From the policy's point of view, two states can look almost identical in observation space while the chair reacts differently because the spring load, relative handle orientation, or contact mode is different. The useful names for this failure mode are `contact-rich hybrid dynamics`, `stiff non-smooth dynamics`, `partial observability`, and `state aliasing`.

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

## Run Lineage

| Family | Result |
|---|---|
| Kinematic visual demo, May 15, 2026 | Good-looking demo only; no physical chair dynamics. |
| Handle-grip walking, May 15, 2026 | Taught the walking policy to keep hands near handle targets. |
| Dynamic free-chair attempts, May 15, 2026 | Added passive wheelchair dynamics, contact rewards, and invalid-contact penalties; tended to veer or collapse. |
| Standing bridge runs, May 16, 2026 | Tried to make the robot stand with the chair/handles before walking; several startup/ragdoll diagnostics were captured. |
| Minimal X-rail runs, May 17, 2026 | Simplified the problem to forward/back chair motion; exposed backward-walking and off-center pushing exploits. |
| PhysX rail diagnostic, May 18, 2026 | Replaced kinematic rail clamp with real prismatic articulation so yaw reaction torque could be measured. |
| PhysX rail soft-attachment run, May 18, 2026 | Current stable large-env setup; not yet learning strong forward pushing. |

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

The next likely changes should expose or reduce the hidden attachment dynamics before adding more reward shaping:

1. Add observations for relative hand-handle velocity, relative hand-handle orientation, soft-attachment force norm, left/right force balance, and rail reaction torque.
2. Add explicit TensorBoard metrics for physical values: wheelchair `vx_mps`, rail yaw torque, hand-handle position error, hand-handle orientation error, and soft-attachment force.
3. Run a controlled comparison between the hard joint, the current soft attachment, and a stiffer bounded spring using the same checkpoint and deterministic playback.
4. Only after that, tune rewards such as hand-handle axis alignment, force symmetry, or yaw-torque penalty.
