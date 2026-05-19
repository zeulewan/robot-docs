# G1 Wheelchair Push Policy

This page is the working summary for the wheelchair-push policy experiments. Keep current decisions here and move detailed chronological notes to the [archive](archive.md).

## Current Status

| Item | Value |
|---|---|
| Active training task | `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-1mps-Fast-Lean-Velocity-Progress-Push-Attached-Hard` |
| Last rendered playback | Reproduced 2 m/s hard-attach reference: `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-Fast-Lean-Velocity-Progress-Push-Attached-Hard`, `model_13300.pt` |
| Main config | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/wheelchair_push_env_cfg.py` |
| Observation helpers | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/observations.py` |
| Attachment helper | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/events.py` |
| Warm-start checkpoint | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_fast_lean_hard_attach_push_attached/2026-05-18_19-47-36_hard_attach_loose_guard_2048_from_13249/model_13300.pt` |
| Preserved 2 m/s visual reference | `Unitree-G1-29dof-Wheelchair-Minimal-PhysX-Rail-Fast-Lean-Velocity-Progress-Push-Attached-Hard`, `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_fast_lean_hard_attach_push_attached/2026-05-18_19-47-36_hard_attach_loose_guard_2048_from_13249/model_13300.pt` |
| Current run | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_fast_lean_hard_attach_push_attached/2026-05-19_03-18-18_hard_attach_1mps_fastlean_long_from_13249_may19` |
| Failed branch kept for comparison | `logs/rsl_rl/unitree_g1_29dof_wheelchair_minimal_physx_rail_1mps_yaw_torque_hard_attach_push_attached/2026-05-18_20-34-46_hard_1mps_yawtorque_from_fastlean_13300` |
| Latest archived playback | `logs/demos/unitree-wheelchair-physx-rail-fast-lean-hard-attach-push-attached_model_13300_slow_revolve_best_20260519_021951/model_13300_slow_revolve_best.mp4` |
| Summary last updated | May 19, 2026, 03:20 Toronto |
| Training tmux | `wheelchair_1mps_fastlean_hard_long` |
| Training env count | `2048` |
| Latest-video page | `https://workstation.tailee9084.ts.net:8002/` |
| Focused TensorBoard | `http://workstation.tailee9084.ts.net:6007/` |

The previous soft-attachment run was stopped at `model_15900.pt` and used as the baseline for the SoftObs branch. That branch is not the visually good reference. The visually useful 2 m/s hard-attachment fast-lean lineage is preserved as the visual reference and warm-start source. The earlier 1 m/s yaw-torque hard branch did not learn useful forward push behavior and is kept only as a failed comparison branch. The current 1 m/s run is a new speed-scaled variant of the reproducible 2 m/s path, not that yaw-torque branch.

Important correction from the May 19 git-history audit and reproduction test: treat the `model_13300.pt` hard-attach checkpoint as a real good visual reference, not as a randomly fragile checkpoint. The exact old command from the `model_13249.pt` source reproduced `model_13300.pt` and `model_13350.pt` byte-for-byte on May 19. The behavior changed in the later restart/modified branches, while the original 2 m/s hard-attach path is still reproducible.

## Current Task Shape

The current reference wheelchair is not free in all directions. It uses the PhysX rail URDF:

`assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair_x_rail.urdf`

That asset fixes `rail_world` and connects the moving `base_link` through a prismatic `rail_x_joint`. The chair can move forward/back along X; yaw, lateral motion, roll, and pitch are constrained by physics instead of the older kinematic root-pose clamp.

The active nonzero reward terms in the current 1 m/s fast-lean hard task are:

| Reward | Weight | Purpose |
|---|---:|---|
| `wheelchair_track_forward_velocity` | `10.0` | Match wheelchair `base_link` forward velocity to the fixed `1.0 m/s` command. |
| `wheelchair_forward_progress` | `3.0` | Reward positive world-X wheelchair movement. |
| `wheelchair_backward_velocity` | `-10.0` | Strongly penalize moving the chair backward. |
| `robot_forward_lean` | `1.0` | Bias the robot root toward a small forward lean while pushing. |

Inherited locomotion, pose, contact, hand-position, and wrist terms are currently set to `0.0` for this task unless explicitly listed above.

## Observation Gap

The policy observes the robot state plus wheelchair-relative state through `wheelchair_root_state_b` and `wheelchair_handle_state_b`. The wheelchair observation includes relative chair position, relative chair velocity, chair forward direction, relative yaw rate, and centerline error. The handle observation includes handle positions in the robot-root frame and hand-to-handle position error.

The hard-reference policy observes the robot state plus wheelchair-relative state, but it does not observe rail reaction force/torque. The current 1 m/s fast-lean task also does not penalize rail yaw torque; `wheelchair_rail_yaw_torque` remains at weight `0.0`. The failed 1 m/s yaw-torque task did penalize yaw torque, but that branch did not reproduce useful forward push behavior.

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
| PhysX rail hard-attach retest, May 18, 2026 | Both hand-handle hard joints moved the chair in playback. The joint-snap warning was fixed, command debug marker noise was removed, and a 2048-env guarded continuation from `model_13249.pt` is running. |

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
| `Episode_Termination/unstable_robot_state` | Guard for finite but runaway robot root state. Nonzero is acceptable while exploring, but sustained high values mean the hard attachment is still destabilizing envs. |
| `Episode_Termination/unstable_wheelchair_state` | Guard for finite but runaway wheelchair `base_link` state. Should stay near `0.0`. |

Do not treat a single train-time reward scalar as proof of success. For this task, the useful validation is deterministic playback from a fixed reset with measured wheelchair forward velocity and rail reaction torque.

## Next Fixes To Discuss

The current evidence says this is not just a reward-weight problem. The policy can keep the attachment numerically stable, but it is not discovering a repeatable way to convert arm motion into forward rail motion.

Best next structural tests:

1. Let the guarded hard-attach run reach at least the first saved checkpoint, then validate with deterministic playback before changing rewards.
2. Watch `unstable_robot_state`; if it stays high or rises, split the guard into linear/angular terms so the failure mode is easier to diagnose.
3. Add direct rail-force / rail-joint observation and TensorBoard metrics. We already print rail wrench in playback, but the policy does not observe that reaction load during training.
4. Add a capped yaw/side-load penalty only after the hard attachment can train long enough to checkpoint. The current hard playback has real forward motion but high rail reaction torque.
5. If both-hand hard attachment keeps producing bad simulator states, try a one-hand rigid or spherical probe to remove the two-arm closed loop while preserving hard force transfer.

Do not keep running the plain SoftObs or SoftObs-Stiff branches without changing the task structure; both failed deterministic validation.
