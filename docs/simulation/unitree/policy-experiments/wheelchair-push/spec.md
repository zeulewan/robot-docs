# Wheelchair Policy Goal Spec

This page is the contract for the wheelchair project. It defines the deliverable, the minimum physical validity requirements, the staged milestones, and the experiment dimensions the training loop is allowed to change.

## Objective

Train a Unitree G1 policy that can manipulate a wheelchair in a physically meaningful way, starting from stable standing while holding the chair and progressing to commanded wheelchair locomotion.

The final target behavior is:

1. Stand while holding the chair.
2. Move the chair forward on command.
3. Move the chair backward on command.
4. Turn the chair left on command.
5. Turn the chair right on command.

Single-policy control is preferred. A staged family of policies is acceptable during development, but the final deliverable should aim to collapse to one command-conditioned controller unless that clearly blocks progress.

## Deliverable

The deliverable is complete when all of the following exist:

1. A best checkpoint or checkpoint lineage with exact task IDs and training commands recorded.
2. Deterministic evaluation rollouts showing stand, forward, backward, left turn, and right turn behavior.
3. A short evaluation summary with the key scalar metrics and the failure cases that still remain.
4. Documentation of the curriculum path that produced the result, including which scaffolds were temporary and which behavior transferred.

## Physical Validity Rules

The final evaluated behavior must satisfy these rules:

1. The wheelchair must be collidable during final evaluation.
2. The final evaluation must not use a fixed-base wheelchair, an X-rail, or a no-collision wheelchair.
3. Allowed steady contact is hands on handles. Torso, pelvis, legs, and non-hand arm links should not be used as support against the chair.
4. The robot should not rely on chair interpenetration, non-physical resets, or hidden kinematic helpers that are absent from the target task.
5. Temporary scaffolds are allowed during curriculum phases if they are explicitly marked as non-deliverable.

## Milestones

| Milestone | Goal | Minimum pass condition |
|---|---|---|
| `M0` | Clean fixed-chair stand | Holds upright for full episode with hands attached and low invalid chair contact. |
| `M1` | Clean free-chair hold | Holds chair with collisions enabled while chair is braked or heavily damped; no torso-bracing exploit. |
| `M2` | Lightly damped hold | Same as `M1`, but with less chair damping and no collapse in deterministic playback. |
| `M3` | Forward creep | Moves chair forward slowly on command without obvious bracing exploit. |
| `M4` | Forward walking | Tracks a stronger forward command with stable gait and chair control. |
| `M5` | Backward control | Moves chair backward on command without collapse or turning exploit. |
| `M6` | Left/right turning | Produces controllable left and right turns with the chair. |
| `M7` | Unified controller | One command-conditioned policy covers stand, forward, backward, left, and right. |

## Acceptance Criteria

These are the default pass gates for phase advancement unless a run shows a clearly better criterion is needed.

For standing and hold phases:

1. `time_out` should be high enough that most deterministic eval episodes finish cleanly.
2. `bad_orientation` and `base_height` resets should be rare.
3. `wheelchair_invalid_contact` should stay near zero in the collidable task.
4. Rollouts should not show the torso or hips using the chair as a support surface.

For `M0` specifically, the fixed-chair hard-attachment scaffold currently treats the same-side `*_wrist_yaw_link` as acceptable handle-contact body alongside the rubber hand. This is deliberate: the hard attachment masks direct hand-handle collision, so strict hand-only contact at this phase produces a false invalid-contact failure at reset. Later free-chair phases should restore stricter contact expectations.

For motion phases:

1. The commanded wheelchair motion should match the observed chair motion directionally and approximately in magnitude.
2. Backward motion should not be secretly solved by turning and rolling forward in world frame.
3. Turning should not be secretly solved by sliding sideways or exploiting constraint geometry.
4. All four wheels should stay plausibly grounded unless a specific maneuver makes brief unloading unavoidable.

For the final controller:

1. The same checkpoint should handle stand, forward, backward, left, and right commands in deterministic playback.
2. The policy should remain physically valid under collisions-enabled evaluation.
3. The result should be stable enough that the remaining work is gait polish and robustness, not basic task completion.

## Allowed Scaffolds

These are acceptable temporary scaffolds during the curriculum:

1. Fixed wheelchair root for initial standing.
2. Braked wheelchair or strong damping for early free-chair hold.
3. Ground-plane lock for height, roll, and pitch.
4. Reduced command set, such as stand-only or forward-only.
5. Warm-starting from an earlier checkpoint when the task changes.

These are not acceptable as final evaluation conditions:

1. No-collision wheelchair.
2. Fixed-base wheelchair.
3. X-rail constraint.
4. Manually hidden support geometry that the final task does not have.

## What The Loop May Iterate On

The auto-research loop is allowed to vary:

1. Reward shaping:
   invalid chair contact, hand-handle load, chair root drift, chair yaw/lateral motion, robot orientation, base height, action smoothness, joint deviation, energy, and command tracking.
2. Observation set:
   chair root state, chair velocity, hand-handle relative pose, hand-handle relative velocity, handle wrench, and related direct chair state terms.
3. Curriculum structure:
   fixed to braked to damped to free, command ranges, phase order, and advancement thresholds.
4. Reset and initialization:
   robot pose, arm pose, chair pose, reset noise, and attachment startup procedure.
5. Action parameterization:
   joint action scales, arm and wrist freedom, waist freedom, and whether specific joints are temporarily constrained.
6. PPO settings:
   full resume versus actor-only warm start, critic reset, exploration std, learning rate, clip range, env count, rollout length, and minibatch structure.
7. Contact rules:
   allowed-contact body lists, invalid-contact penalty weights, and whether the task should read contact directly or through derived penalties.

## What The Loop Should Record Each Iteration

Every iteration should leave behind:

1. A one-line hypothesis.
2. The exact task/config change.
3. The exact train command.
4. The checkpoint lineage used to initialize it.
5. A short deterministic evaluation result.
6. A decision:
   continue, branch, revert, or promote to next milestone.

## Example Experiment Types

These are valid examples of loop iterations:

1. Retrain `M0` from scratch with collidable chair and invalid-contact penalty active from the first step.
2. Compare fixed-chair standing with and without per-axis handle-force penalties.
3. Compare free-chair hold with heavy damping versus pose tether versus braked chair.
4. Add chair-state observations to a stable standing checkpoint and test whether transfer to free-chair hold improves.
5. Keep the same task but change only resume mode:
   full PPO continuation versus actor-only warm start with critic reset.
6. Introduce forward creep before full forward walking to see whether command tracking needs a smaller first motion target.
7. Split turning into its own phase before merging into a unified controller.

## Current Status

The current retained `M0` solution is no longer the original direct-observation branch.

1. The direct-observation `M0` loop was superseded. It could improve the fixed-chair score somewhat, but it kept inheriting the fixed-chair bracing exploit from the `900`-dim warm-start source.
2. The retained `M0` solution is now the observed-state branch:
   `Unitree-G1-29dof-Wheelchair-Scratch-M0-CollidableStand-Observed`.
3. That branch warm-started from the relaxed attached standing checkpoint and reached deterministic `M0` eval with:
   `m0_score = 1.0`, `clean_hold_rate = 1.0`, `invalid_contact_rate = 0.0`.
4. The next milestone is `Phase 1A` damped release on the same observed-state branch.
5. The first bounded `Phase 1A` transfer from the clean observed `M0` checkpoint did not reintroduce torso bracing, but it failed through bilateral handle invalid-contact once the chair started moving.
6. A temporary early-release scaffold that allows same-side `*_wrist_yaw_link` handle contact, matching the `M0` logic, removed the fake handle-contact blow-up and produced a usable `Phase 1A` branch:
   `clean_hold_rate = 0.8046875`, `invalid_contact_rate = 0.1953125`, `time_out_rate = 0.875`.
7. The dominant remaining `Phase 1A` failure after that change is no longer handle semantics; it is wheelchair base contact plus release-phase drift and mild balance loss.
8. A follow-up base-only invalid-contact penalty on top of the relaxed-handle `Phase 1A` branch was a regression. It reduced neither drift nor stability cleanly and dropped deterministic release eval to:
   `clean_hold_rate = 0.421875`, `invalid_contact_rate = 0.578125`, `time_out_rate = 0.75`.
9. A second follow-up that tightened the chair pose and velocity tethering on top of the relaxed-handle branch was also a regression. It overconstrained the release phase, increased wheelchair-base contact again, and dropped deterministic release eval to:
   `clean_hold_rate = 0.4453125`, `invalid_contact_rate = 0.5546875`, `time_out_rate = 0.796875`.
10. The retained `Phase 1A` baseline is therefore still the relaxed-handle observed branch. The current evidence says the next useful lever is not stronger tethering or sharper invalid-contact penalties; it is a lighter release-phase shaping change that reduces drift without pushing the robot back into the chair.
11. The first physically relevant `M1` branch is now a collidable braked-chair task with the same temporary same-side wrist-yaw handle allowance used by `M0`. This removed the earlier handle invalid-contact failure entirely, but the first bounded probe still failed purely through orientation instability:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1-BrakedHold-Observed-RelaxedHandle`
    with deterministic eval
    `invalid_contact_rate = 0.0`, `bad_orientation_rate = 1.0`, `time_out_rate = 0.0`, `m0_score = -0.75`.
12. A follow-up `M1` variant that kept the same collidable relaxed-handle scaffold but switched to the stronger stationary-chair reward set improved stability materially without reintroducing invalid contact:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1-BrakedStationary-Observed-RelaxedHandle`
    with deterministic eval
    `invalid_contact_rate = 0.0`, `bad_orientation_rate = 0.9296875`, `time_out_rate = 0.0703125`, `clean_hold_rate = 0.0703125`, `m0_score = -0.626953125`.
13. The first follow-up `M1` stability sweep kept the same two-hand collidable stationary branch and tried two narrow changes. Neither helped enough to retain:
    - reduced arm and wrist action freedom:
      `bad_orientation_rate = 0.9453125`, `time_out_rate = 0.0546875`, `clean_hold_rate = 0.0546875`, `m0_score = -0.654296875`
    - stronger upright and low-motion regularization:
      `bad_orientation_rate = 0.9296875`, `time_out_rate = 0.0703125`, `clean_hold_rate = 0.0703125`, `m0_score = -0.6328125`
14. A same-task full PPO resume from the two-hand stationary collidable checkpoint also did not improve the retained result:
    `bad_orientation_rate = 0.9375`, `time_out_rate = 0.0625`, `clean_hold_rate = 0.0625`, `m0_score = -0.640625`.
15. The first materially better physically valid post-`M0` branch came from changing the scaffold, not the reward weights. A temporary one-hand collidable stationary braked `M1` variant breaks the two-arm closed chain by attaching only the left hand:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1-BrakedStationary-Observed-LeftHand-RelaxedHandle`.
16. The first bounded one-hand run from the same two-hand source checkpoint became the new retained `M1` scaffold with deterministic eval:
    `bad_orientation_rate = 0.2109375`, `invalid_contact_rate = 0.0078125`, `time_out_rate = 0.7890625`, `clean_hold_rate = 0.78125`, `m0_score = 0.6061033082008361`.
17. Continuing that one-hand branch for another short same-task training block stayed physically clean, but it drifted slightly on deterministic eval rather than improving the retained checkpoint. Saved checkpoints from that continuation scored:
    - `model_9800.pt`:
      `bad_orientation_rate = 0.2265625`, `invalid_contact_rate = 0.0078125`, `time_out_rate = 0.7734375`, `clean_hold_rate = 0.765625`, `m0_score = 0.595680835545063`
    - `model_9836.pt`:
      `bad_orientation_rate = 0.2421875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.75`, `clean_hold_rate = 0.75`, `m0_score = 0.5625`
18. The retained best physically valid post-`M0` branch is therefore still the first one-hand collidable stationary braked `M1` checkpoint, not the later continuation. The current evidence is that the main blocker on the two-hand physical branch is the closed-chain attachment geometry, not missing contact penalties.
19. The next successful curriculum step keeps the retained left-hand hard attachment and reintroduces the right hand as a bounded soft assist instead of a second hard joint:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1b-BrakedStationary-Observed-LeftHardRightSoft-RelaxedHandle`.
20. That `M1b` stage preserves the same `585`-dim observation space as the retained one-hand branch, so the one-hand checkpoint can be evaluated there directly. That immediate transfer is the current best physically valid free-chair hold result so far:
    `bad_orientation_rate = 0.1015625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.8984375`, `clean_hold_rate = 0.8984375`, `m0_score = 0.822265625`.
21. A short same-stage warm-start continuation from the same checkpoint did not improve that immediate-transfer result. Deterministic eval after 20 iterations gave:
    - `model_9800.pt`:
      `bad_orientation_rate = 0.125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.8828125`, `clean_hold_rate = 0.875`, `m0_score = 0.7890625`
    - `model_9806.pt`:
      `bad_orientation_rate = 0.171875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.828125`, `clean_hold_rate = 0.828125`, `m0_score = 0.69921875`
22. The retained best `M1`/early-`M2` scaffold is therefore now the immediate-transfer `M1b` result, not the continuation. The evidence so far says the right direction is staged second-hand reintroduction with bounded compliance, while naïve continued PPO updates on that stage still destabilize orientation.
23. The next promotion attempt introduced an explicit damped dynamic-chair stage:
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-DampedStationary-Observed-LeftHardRightSoft-RelaxedHandle`.
    This keeps the retained `M1b` left-hard/right-soft grip scaffold, but swaps the braked chair for a lightly damped dynamic chair with `linear_damping = 0.15`, `angular_damping = 0.15`, and stronger chair-stationary shaping.
24. Immediate transfer of the retained `M1b` checkpoint into that `M2` stage was physically clean but materially worse on deterministic eval:
    `bad_orientation_rate = 0.515625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.484375`, `clean_hold_rate = 0.484375`, `m0_score = 0.09765625`.
25. A short `20`-iteration warm-start continuation on the same `M2` stage did not recover that drop. Both saved checkpoints evaluated to the same deterministic result:
    - `model_9800.pt`:
      `bad_orientation_rate = 0.515625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.484375`, `clean_hold_rate = 0.484375`, `m0_score = 0.09765625`
    - `model_9806.pt`:
      `bad_orientation_rate = 0.515625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.484375`, `clean_hold_rate = 0.484375`, `m0_score = 0.09765625`
26. The current read is that this first damped-chair `M2` promotion is a failed branch, not a retained milestone. It removes invalid contact cleanly, but the stability drop is too large, and short warm-start PPO updates did not move it. The retained scaffold therefore remains the immediate-transfer `M1b` result until a gentler `M2` transition is found.
27. The next bridge attempt inserted a medium-damped dynamic-chair stage instead of jumping directly from braked `M1b` to light-damped `M2`:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1c-MediumDampedStationary-Observed-LeftHardRightSoft-RelaxedHandle`.
    This stage keeps the retained `M1b` left-hard/right-soft scaffold and reward shaping unchanged, and only reduces the chair damping partway to the failed `M2` values.
28. Immediate transfer of the retained `M1b` checkpoint into `M1c` was materially better than the failed `M2` jump while staying physically clean:
    `bad_orientation_rate = 0.375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.625`, `clean_hold_rate = 0.625`, `m0_score = 0.34375`.
29. A short `20`-iteration warm-start continuation on `M1c` improved that bridge stage modestly without reintroducing any invalid contact:
    - `model_9800.pt`:
      `bad_orientation_rate = 0.3515625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.65625`, `clean_hold_rate = 0.6484375`, `m0_score = 0.392578125`
    - `model_9806.pt`:
      `bad_orientation_rate = 0.34375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.65625`, `clean_hold_rate = 0.65625`, `m0_score = 0.3984375`
30. The current read is that `M1c` is the best dynamic-chair bridge so far, and it clearly narrows the gap from `M1b` to a moving chair better than the old `M2` attempt. But it is still materially worse than the retained braked `M1b` scaffold, so it should be treated as a provisional intermediate stage rather than a promoted new baseline.
31. A longer same-stage continuation from the retained `M1c` `model_9806.pt` did not preserve the improved online training statistics in deterministic eval. The saved `model_9845.pt` checkpoint regressed to:
    `bad_orientation_rate = 0.3828125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.6171875`, `clean_hold_rate = 0.6171875`, `m0_score = 0.330078125`.
    So the retained `M1c` result remains the earlier short-run `model_9806.pt`, not the longer continuation.
32. Changing only the resume mode on `M1c` helped. A bounded full-PPO resume from the retained `M1c` `model_9806.pt` produced a better deterministic bridge checkpoint:
    `model_9825.pt` with
    `bad_orientation_rate = 0.328125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.671875`, `clean_hold_rate = 0.671875`, `m0_score = 0.42578125`.
    This is the current retained `M1c` checkpoint. The evidence is that optimizer state matters on this stage; actor-only warm starts were leaving some performance on the table.
33. Re-testing the lighter-damped `M2` stage from that improved retained `M1c` checkpoint helped the raw transfer a little but still did not make `M2` promotable. Immediate transfer of `model_9825.pt` into `M2` reached:
    `bad_orientation_rate = 0.4921875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5078125`, `clean_hold_rate = 0.5078125`, `m0_score = 0.138671875`.
    That is better than the earlier `M2` transfer from the weaker `M1b` source, but still materially behind `M1c`.
34. A short `20`-iteration warm-start continuation on `M2` from the improved `M1c` `model_9825.pt` regressed again instead of consolidating the gain. The saved `model_9844.pt` checkpoint evaluated to:
    `bad_orientation_rate = 0.5390625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.4609375`, `clean_hold_rate = 0.4609375`, `m0_score = 0.056640625`.
    So the light-damped `M2` task is still not the next retained milestone. The current best ladder is `M1b` braked hold, then `M1c` medium-damped bridge, with `M2` still blocked by stage design rather than simple checkpoint quality.
35. To separate light damping from the stronger `M2` stationary reward shaping, a second light-damped hold probe was added:
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-LightDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`.
    This uses the same light-damped chair as `M2`, but keeps the `M1b`/`M1c` hold reward scaffold unchanged.
36. Immediate transfer from the retained `M1c model_9825.pt` into that isolated light-damped hold task matched the prior `M2` transfer:
    `bad_orientation_rate = 0.4921875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5078125`, `clean_hold_rate = 0.5078125`, `m0_score = 0.138671875`.
37. A short `20`-iteration warm-start continuation on the isolated light-damped hold task also failed to retain the gain:
    `model_9844.pt` with
    `bad_orientation_rate = 0.5078125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.4921875`, `clean_hold_rate = 0.4921875`, `m0_score = 0.111328125`.
    This suggests the main cliff is the chair dynamics/damping drop itself, not only the stronger `M2` stationary reward weights.
38. To narrow that dynamics cliff further, a midpoint bridge stage was added between `M1c` and the light-damped tasks:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1d-TransitionDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`.
    It keeps the same left-hard/right-soft hold scaffold and reward shaping as `M1b`/`M1c`, but uses a transition wheelchair with `linear_damping = 0.25`, `angular_damping = 0.25`, and wheel/caster drive stiffness `1.75`.
39. Immediate transfer from the retained `M1c model_9825.pt` into `M1d` was better than both light-damped branches while staying physically clean:
    `bad_orientation_rate = 0.4453125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5546875`, `clean_hold_rate = 0.5546875`, `m0_score = 0.220703125`.
    That is still materially behind retained `M1c`, but it shows the dynamics cliff is at least partly smoothable with a finer damping ladder.
40. A short `20`-iteration warm-start continuation on `M1d` did not consolidate that gain. The saved `model_9844.pt` checkpoint regressed to:
    `bad_orientation_rate = 0.484375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.515625`, `clean_hold_rate = 0.515625`, `m0_score = 0.15234375`.
41. Changing only the continuation mode on `M1d` helped, just as it had on `M1c`. A bounded full-PPO resume from the retained `M1c model_9825.pt` produced a better `M1d` checkpoint:
    `model_9844.pt` with
    `bad_orientation_rate = 0.4375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5625`, `clean_hold_rate = 0.5625`, `m0_score = 0.234375`.
    That is a modest gain over the raw `M1d` transfer, but it is the current retained `M1d` result and shows that optimizer state still matters on the transition stages.
42. Using that stronger retained `M1d` checkpoint directly on the old light-damped hold stage still did not lift the real blocker. Immediate transfer into
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-LightDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
    came back at:
    `bad_orientation_rate = 0.5078125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.4921875`, `clean_hold_rate = 0.4921875`, `m0_score = 0.111328125`.
    So the `0.25 -> 0.15` dynamics gap was still too large.
43. To narrow that remaining gap, a second finer transition stage was added:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1e-LightTransitionDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`.
    It keeps the same hold scaffold and reward shaping, but uses a lighter transition wheelchair with `linear_damping = 0.20`, `angular_damping = 0.20`, and wheel/caster drive stiffness `1.4`.
44. Immediate transfer from the retained `M1d model_9844.pt` into `M1e` improved over the failed light-damped stage while staying physically clean:
    `bad_orientation_rate = 0.46875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.53125`, `clean_hold_rate = 0.53125`, `m0_score = 0.1796875`.
    That is still below retained `M1d`, but it is meaningfully better than the old `M2` raw transfer and confirms that the ladder can still be smoothed further by tightening the dynamics step.
45. A bounded full-PPO continuation on `M1e` did not retain that improvement. Deterministic eval of the saved checkpoints regressed below the raw transfer:
    - `model_9850.pt`:
      `bad_orientation_rate = 0.484375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.515625`, `clean_hold_rate = 0.515625`, `m0_score = 0.15234375`
    - `model_9863.pt`:
      `bad_orientation_rate = 0.5`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5`, `clean_hold_rate = 0.5`, `m0_score = 0.125`
46. Changing only the continuation mode on `M1e` helped, just as it had on `M1d`. A short model-only continuation from the retained `M1d model_9844.pt` produced a better `M1e` checkpoint:
    - `model_9850.pt`:
      `bad_orientation_rate = 0.484375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.515625`, `clean_hold_rate = 0.515625`, `m0_score = 0.15234375`
    - `model_9863.pt`:
      `bad_orientation_rate = 0.453125`, `base_height_rate = 0.0`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.546875`, `clean_hold_rate = 0.546875`, `m0_score = 0.20703125`
    The retained `M1e` checkpoint is now `model_9863.pt`.
47. Re-testing the fully light-damped hold stage from that retained `M1e model_9863.pt` still did not lift the real blocker. Immediate transfer into
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-LightDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
    came back at:
    `bad_orientation_rate = 0.5078125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.4921875`, `clean_hold_rate = 0.4921875`, `m0_score = 0.111328125`.
    So the `0.20 -> 0.15` dynamics gap was still too large.
48. To narrow that final remaining gap, a third and finer transition stage was added:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1f-FineTransitionDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`.
    It keeps the same hold scaffold and reward shaping, but uses a finer transition wheelchair with `linear_damping = 0.175`, `angular_damping = 0.175`, and wheel/caster drive stiffness `1.2`.
49. Immediate transfer from the retained `M1e model_9863.pt` into `M1f` stayed physically clean but was still only a midpoint result:
    `bad_orientation_rate = 0.4921875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5078125`, `clean_hold_rate = 0.5078125`, `m0_score = 0.138671875`.
50. A short model-only continuation on `M1f` from that retained `M1e` checkpoint did retain the new rung. The saved `model_9882.pt` checkpoint evaluated to:
    `bad_orientation_rate = 0.4375`, `base_height_rate = 0.0078125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5625`, `clean_hold_rate = 0.5625`, `m0_score = 0.228515625`.
    This is the current retained `M1f` checkpoint.
51. Using that stronger retained `M1f model_9882.pt` directly on the fully light-damped hold stage helped only slightly. Immediate transfer into
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-LightDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
    reached:
    `bad_orientation_rate = 0.5`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5`, `clean_hold_rate = 0.5`, `m0_score = 0.125`.
    That is marginally better than the old `0.111328125` light-damped transfer, but still not a promotable `M2` result.
52. A short model-only continuation on the same `M2` stage from retained `M1f model_9882.pt` did not retain the slight gain. One saved checkpoint failed to emit a valid eval result file, and the surviving deterministic eval regressed:
    - `model_9900.pt`:
      no valid metrics file emitted by the benchmark wrapper
    - `model_9901.pt`:
      `bad_orientation_rate = 0.515625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.484375`, `clean_hold_rate = 0.484375`, `m0_score = 0.09765625`
53. The current read is now concrete. `M1d`, `M1e`, and `M1f` all became usable retained bridge rungs once continuation mode was softened to model-only, but the fully light-damped `M2` stage is still blocked even with the improved source checkpoints. The next likely lever is another task-design change at the light-damped boundary, not more continuation on the current `M2` setup.
54. To isolate that remaining `M1f -> M2` cliff, two split boundary variants were added:
    - `Unitree-G1-29dof-Wheelchair-Scratch-M2a-LightBodyDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
      changes only the chair body damping to the `M2` level (`linear_damping = 0.15`, `angular_damping = 0.15`) while keeping the retained `M1f` wheel-drive stiffness `1.2`.
    - `Unitree-G1-29dof-Wheelchair-Scratch-M2b-SoftDriveTransitionHold-Observed-LeftHardRightSoft-RelaxedHandle`
      keeps the retained `M1f` body damping (`0.175`) while dropping only the wheel/caster drive stiffness to the `M2` level (`1.0`).
55. Immediate transfer from retained `M1f model_9882.pt` into those split variants showed the boundary is asymmetric:
    - `M2a` raw transfer:
      `bad_orientation_rate = 0.484375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.515625`, `clean_hold_rate = 0.515625`, `m0_score = 0.15234375`
    - `M2b` raw transfer:
      `bad_orientation_rate = 0.46875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.53125`, `clean_hold_rate = 0.53125`, `m0_score = 0.1796875`
    The stiffness drop alone is therefore less damaging than the body-damping drop alone.
56. A short model-only continuation on `M2b` did not retain the raw-transfer gain. The saved `model_9901.pt` checkpoint evaluated to:
    `bad_orientation_rate = 0.4765625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.53125`, `clean_hold_rate = 0.5234375`, `m0_score = 0.173828125`.
    So `M2b` is useful as a probe, but its retained best is still the immediate-transfer result rather than the continuation.
57. A short model-only continuation on `M2a` did help a little, but not enough to beat the stronger `M2b` raw transfer. Direct deterministic eval of the saved checkpoints came back at:
    - `model_9900.pt`:
      `bad_orientation_rate = 0.484375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.515625`, `clean_hold_rate = 0.515625`, `m0_score = 0.15234375`
    - `model_9901.pt`:
      `bad_orientation_rate = 0.4765625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5234375`, `clean_hold_rate = 0.5234375`, `m0_score = 0.166015625`
58. The current read is now narrower. The wheel-drive stiffness drop is not the main blocker at the light-damped boundary; the chair body-damping drop is. The strongest boundary result below retained `M1f` is now the raw `M2b` transfer at `m0_score = 0.1796875`. The next useful lever is therefore another finer damping rung or a redesigned body-damping transition, not more same-task continuation on `M2`.
59. A finer body-damping rung was then added directly below `M1f`:
    `Unitree-G1-29dof-Wheelchair-Scratch-M1g-BodyTransitionDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`.
    This stage keeps the retained `M1f` wheel-drive stiffness `1.2` and lowers only the chair body damping partway to `M2a`, using `linear_damping = 0.1625` and `angular_damping = 0.1625`.
60. Immediate transfer from retained `M1f model_9882.pt` into `M1g` was physically clean and matched the earlier best `M2b` probe:
    `bad_orientation_rate = 0.46875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.53125`, `clean_hold_rate = 0.53125`, `m0_score = 0.1796875`.
61. A short model-only continuation on `M1g` did retain that rung locally. The saved checkpoints evaluated to:
    - `model_9900.pt`:
      `bad_orientation_rate = 0.5`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5`, `clean_hold_rate = 0.5`, `m0_score = 0.125`
    - `model_9901.pt`:
      `bad_orientation_rate = 0.4609375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5390625`, `clean_hold_rate = 0.5390625`, `m0_score = 0.193359375`
    So the retained same-stage `M1g` checkpoint is `model_9901.pt`.
62. But that same-stage improvement did not improve the true body-damping boundary. Immediate transfer from retained `M1g model_9901.pt` into
    `Unitree-G1-29dof-Wheelchair-Scratch-M2a-LightBodyDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
    fell back to:
    `bad_orientation_rate = 0.5`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5`, `clean_hold_rate = 0.5`, `m0_score = 0.125`.
63. That changes the lesson from this branch. Same-stage bridge improvement is not sufficient as a retention criterion by itself, because `M1g` improved its own deterministic score while failing to improve downstream transfer into `M2a`. Future bridge-stage acceptance should therefore use downstream-stage transfer as a gate, not only same-stage deterministic eval.
64. The bridge harness was then upgraded to support `--evaluate-all-checkpoints`, so bounded bridge runs can score every saved checkpoint and keep the one with the best downstream transfer metric rather than automatically taking the latest checkpoint. Re-scoring the existing `M1g` run with that rule changed the retained result:
    - `model_9900.pt` was the best downstream-transfer checkpoint, not `model_9901.pt`
    - same-stage `M1g`: `m0_score = 0.125`
    - downstream `M2a`: `bad_orientation_rate = 0.4765625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5234375`, `clean_hold_rate = 0.5234375`, `m0_score = 0.166015625`
65. A fresh downstream-aware `M1g` continuation from retained `M1f model_9882.pt` with lower exploration (`policy_std = 0.005`) improved the real body-damping boundary further. The selected checkpoint from run
    `2026-05-26_05-52-09_m1g_bridge_std005_from_m1f_9882`
    was `model_9900.pt`, with:
    - same-stage `M1g`: `bad_orientation_rate = 0.4765625`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5234375`, `clean_hold_rate = 0.5234375`, `m0_score = 0.166015625`
    - downstream `M2a`: `bad_orientation_rate = 0.453125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.546875`, `clean_hold_rate = 0.546875`, `m0_score = 0.20703125`
66. That `M1g std=0.005 model_9900.pt` checkpoint is now the best pre-`M2a` bridge source we have seen. It beats the older raw `M2b` probe (`0.1796875`), the earlier `M2a` continuation from `M1f` (`0.166015625`), and the first same-stage-selected `M1g` result.
67. Using that downstream-selected `M1g model_9900.pt` as the source for a short low-noise `M2a` continuation lifted the actual light-body-damped stage itself. In run
    `2026-05-26_05-57-38_m2a_from_m1g9900_std005`,
    the best selected checkpoint was `model_9900.pt` with:
    `bad_orientation_rate = 0.4453125`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5546875`, `clean_hold_rate = 0.5546875`, `m0_score = 0.220703125`.
    The later saved `model_9919.pt` regressed to `m0_score = 0.15234375`, confirming again that earliest downstream-clean checkpoints can be better than the latest checkpoint on these delicate bridge stages.
68. The current retained ladder is therefore no longer just a sequence of stage-local optima. The best known path is now:
    - retained `M1f model_9882.pt`
    - downstream-selected `M1g std=0.005 model_9900.pt`
    - retained `M2a model_9900.pt` from `m2a_from_m1g9900_std005`
    with zero invalid chair contact throughout.
69. Starting from that retained `M2a model_9900.pt`, the remaining wheel-drive stiffness drop into full
    `Unitree-G1-29dof-Wheelchair-Scratch-M2-LightDampedHold-Observed-LeftHardRightSoft-RelaxedHandle`
    was then retried with the downstream-aware checkpoint-selection harness and lower exploration (`policy_std = 0.005`).
70. In run
    `2026-05-26_06-03-42_m2_from_m2a9900_std005`,
    the selected checkpoint was `model_9900.pt`, not the later `model_9919.pt`. The retained `M2` result came back at:
    `bad_orientation_rate = 0.4609375`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.5390625`, `clean_hold_rate = 0.5390625`, `m0_score = 0.193359375`.
    The later checkpoint regressed to:
    `bad_orientation_rate = 0.46875`, `invalid_contact_rate = 0.0`, `time_out_rate = 0.53125`, `clean_hold_rate = 0.53125`, `m0_score = 0.1796875`.
71. This is the first retained full-`M2` checkpoint on the current observed left-hard/right-soft ladder. It materially beats the earlier raw `M2` transfer (`0.125`) and the failed older continuation branch (`0.09765625`), while keeping invalid chair contact at zero.
72. But `M2` is still weaker than the retained `M2a` source (`0.220703125`). So the wheel-drive stiffness boundary is no longer a hard failure, but it is still the next optimization target. The current retained ladder is now:
    - retained `M1f model_9882.pt`
    - downstream-selected `M1g std=0.005 model_9900.pt`
    - retained `M2a model_9900.pt`
    - retained `M2 model_9900.pt`
73. The next branch should start from retained `M2 model_9900.pt` and begin the first real motion curriculum step on top of the current physically clean hold scaffold, rather than revisiting old bridge rungs or latest-checkpoint heuristics.
74. The first motion-stage branch on top of retained `M2 model_9900.pt` was
    `Unitree-G1-29dof-Wheelchair-Scratch-M3-CreepForward-Observed-LeftHardRightSoft-RelaxedHandle`.
    This stage keeps the physically clean left-hard/right-soft hold scaffold, drops the stationary-chair objective, and introduces a small forward wheelchair command (`0.10 m/s`) with direct chair-motion shaping.
75. The original motion-stage scalar was too forgiving. `forward_motion_score` had been using a clipped positive-only forward-velocity ratio, so high-survival rail runs could still look good even when the wheelchair was moving backward. The evaluator was then corrected to use a signed symmetric forward-velocity ratio together with lateral/yaw penalties. After that fix, old `M3` and rail results had to be re-read.
76. Under the corrected directional metric, the retained free-chair `M3` branch is still not a usable forward-motion milestone. Raw transfer from retained `M2 model_9900.pt` into `M3` came back at:
    `forward_motion_score = -0.013570901006460152`, `clean_hold_rate = 0.484375`, `time_out_rate = 0.484375`, `wheelchair_forward_velocity_mean = 0.0001815104780253023`, `wheelchair_forward_velocity_ratio_symmetric = 0.0023294897258020943`.
    A bounded free-chair `M3` continuation improved only slightly:
    `model_9999.pt` with
    `forward_motion_score = 0.025361138582229645`, `clean_hold_rate = 0.515625`, `time_out_rate = 0.515625`, `wheelchair_forward_velocity_mean = 0.003041791496798396`, `wheelchair_forward_velocity_ratio_symmetric = 0.030903536826372147`.
    So the chair was still barely moving.
77. The first rail curriculum branch,
    `Unitree-G1-29dof-Wheelchair-Scratch-M3a-RailCreepForward-Observed-LeftHardRightSoft-RelaxedHandle`,
    was then tested to simplify the motion problem. It solved survival on the rail, but after the scoring fix it was clearly a failed branch: the selected same-stage checkpoint `model_9950.pt` had
    `forward_motion_score = -0.15196037504938428`, `clean_hold_rate = 0.9921875`, `time_out_rate = 0.9921875`, `wheelchair_forward_velocity_mean = -0.07597053050994873`, `wheelchair_forward_velocity_ratio_symmetric = -0.5507431030273438`.
    Downstream transfer from that rail checkpoint back into free-chair `M3` was still effectively zero-motion:
    `forward_motion_score = -0.01093803327530615`, `clean_hold_rate = 0.4921875`, `wheelchair_forward_velocity_mean = 0.0001280088904313743`.
78. A second rail curriculum branch,
    `Unitree-G1-29dof-Wheelchair-Scratch-M3b-RailDenseForward-Observed-LeftHardRightSoft-RelaxedHandle`,
    widened the forward-velocity well and made dense chair progress dominate the rail stage. That improved the training signal, but it still did not produce a valid forward-motion milestone. The selected same-stage checkpoint `model_9999.pt` still moved backward on the rail:
    `forward_motion_score = 0.04777805805206303`, `clean_hold_rate = 0.984375`, `time_out_rate = 0.984375`, `wheelchair_forward_velocity_mean = -0.07038901746273041`, `wheelchair_forward_velocity_ratio_symmetric = -0.5341796875`.
    Its downstream transfer into free-chair `M3` was slightly positive but still tiny:
    `forward_motion_score = 0.015163969621062322`, `clean_hold_rate = 0.515625`, `wheelchair_forward_velocity_mean = 0.0014841918600723147`, `wheelchair_forward_velocity_ratio_symmetric = 0.015274673700332642`.
79. The current motion-stage read is therefore straightforward. The retained hold scaffold through `M2` is physically clean, but the first forward-motion curriculum is still blocked. The corrected metric shows both rail branches failed to produce real forward chair motion, and the best free-chair `M3` result is still only marginally above zero. The next useful lever is not more reward nudging on the same rail tasks; it should be a different motion-stage scaffold or command structure that cannot hide behind backward or near-stationary solutions.
80. A third rail probe then tested whether the failure was mostly the shape of the motion reward itself:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3c-RailSignedForward-Observed-LeftHardRightSoft-RelaxedHandle`.
    This branch removed the soft exponential chair-velocity matching term entirely and replaced it with strictly directional shaping: strong positive `wheelchair_forward_progress`, linear `wheelchair_backward_velocity_l1`, and the same rail constraint.
81. That probe was a clean negative result. The rail stage itself still settled into backward motion:
    - selected same-stage checkpoint `model_9900.pt`:
      `forward_motion_score = -0.09549030592315827`, `clean_hold_rate = 0.9921875`, `time_out_rate = 0.9921875`, `wheelchair_forward_velocity_mean = -0.06506837904453278`, `wheelchair_forward_velocity_ratio_symmetric = -0.49263304471969604`
    Later checkpoints such as `model_9999.pt` stayed fully stable on the rail but still moved backward:
      `forward_motion_score = -0.147451005372568`, `wheelchair_forward_velocity_mean = -0.06927454471588135`.
82. Downstream transfer from `M3c` back into free-chair `M3` also did not improve. The best selected downstream checkpoint was again `model_9900.pt` with:
    `forward_motion_score = -0.011835230141878147`, `clean_hold_rate = 0.484375`, `time_out_rate = 0.484375`, `wheelchair_forward_velocity_mean = 0.00040248059667646885`, `wheelchair_forward_velocity_ratio_symmetric = 0.004518650472164154`.
    That is effectively the same as the raw retained `M2 -> M3` transfer and confirms that reward-shape cleanup alone is not enough on the current left-hard/right-soft rail bridge.
83. The motion-stage blocker is therefore narrower now. The clean hold ladder through `M2` is still valid, but the current `M3` family does not bridge into actual chair propulsion. The next useful branch should borrow more aggressively from the older minimal successful motion scaffolds rather than keep iterating inside the current `M3a/M3b/M3c` structure. The most likely levers are a more minimal motion reward set, larger action authority, and possibly a temporarily stronger motion-phase hand constraint.
84. A fourth motion probe then borrowed more directly from the older minimal successful scaffolds:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3d-GroundLockHeavyDampedForward-Observed-LeftHardRightSoft-RelaxedHandle`.
    This branch replaced the rail with a ground-lock and heavy planar damping scaffold, increased leg/waist/arm/wrist action authority substantially, relaxed base-height and orientation terminations, strengthened the right soft hand attachment, and reduced the reward set to a sparse motion core around forward progress, backward penalty, lateral/yaw penalties, lean bias, and low-weight hand geometry.
85. The bounded `M3d` run was stable on its own constrained stage, but it exposed a new failure mode instead of solving motion. The selected same-stage checkpoint was `model_9900.pt` with:
    `forward_motion_score = 0.2117772144381888`, `clean_hold_rate = 0.6484375`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.02916671335697174`,
    but also `invalid_contact_rate = 0.3515625`, dominated by
    `wheelchair_base_robot_contact = 0.34375` and
    `wheelchair_right_handle_invalid_contact = 0.140625`.
    So the constrained stage itself was already learning contact abuse instead of a clean push.
86. Downstream transfer from `M3d` back into the real free-chair `M3` did not help. The best selected downstream checkpoint was again `model_9900.pt` with:
    `forward_motion_score = 0.005037643201649188`, `clean_hold_rate = 0.5`, `time_out_rate = 0.5`, `bad_orientation_rate = 0.5`,
    `wheelchair_forward_velocity_mean = 0.0013121002120897174`, and `invalid_contact_rate = 0.0`.
    Later checkpoints `model_9950.pt` and `model_9999.pt` were worse downstream. This makes `M3d` a discard: it is weaker than the earlier bounded free-chair `M3` continuation (`0.025361138582229645`) and weaker than the denser rail branch `M3b` (`0.015163969621062322`) on the actual forward-motion metric.
87. The current motion-stage diagnosis is now sharper. Minimal reward cleanup, larger action authority, and a ground-lock/heavy-damping scaffold can produce a stable constrained-stage gait, but under the current observed left-hard/right-soft setup they still do not transfer into real free-chair chair propulsion and they reopen chair-contact exploitation. The next branch should change the motion-stage constraint structure itself rather than keep tuning within `M3b/M3c/M3d`.
88. That next branch was `M3e`:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3e-GroundLockHeavyDampedForward-Observed-BothHard-RelaxedHandle`.
    It keeps the same minimal `M3d` motion scaffold but replaces the left-hard/right-soft grip with both-hand hard attachment during the constrained motion bridge. This was a useful correction. Same-stage `M3e` became almost perfectly clean:
    - selected checkpoint `model_9950.pt`
    - `forward_motion_score = 0.28799832941731435`
    - `clean_hold_rate = 0.9921875`
    - `time_out_rate = 1.0`
    - `bad_orientation_rate = 0.0`
    - `invalid_contact_rate = 0.0078125`
    So the right-hand drift/bracing failure from `M3d` was largely removed.
89. But forcing transfer from `M3e` back into the older free-chair soft-right `M3` task still underperformed. The best selected downstream checkpoint was `model_9950.pt` with:
    `forward_motion_score = 0.013585472479462624`, `clean_hold_rate = 0.5078125`, `time_out_rate = 0.5078125`, `bad_orientation_rate = 0.4921875`, and `invalid_contact_rate = 0.0`.
    That is better than `M3d`, but still weaker than the earlier bounded free-chair continuation (`0.025361138582229645`). The important lesson is that the old left-hard/right-soft free-chair target had become the wrong curriculum target once the cleaner both-hard motion scaffold was introduced.
90. The next stage therefore promoted the cleaner grip into the freer motion task itself:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3f-FreeYawHeavyDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3f` removes the ground-plane clamp from `M3e` while keeping the both-hard grip and heavy planar damping. This is the first strong positive result on the current motion ladder. Starting from retained `M3e model_9950.pt`, the bounded `M3f` continuation selected `model_10049.pt` with:
    `forward_motion_score = 0.3852109075058252`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.014253754168748856`,
    `wheelchair_lateral_velocity_abs_mean = 0.0014836001209914684`,
    and `wheelchair_yaw_velocity_abs_mean = 0.004958887584507465`.
    This makes `M3f` the new retained motion rung. It is materially better than every prior `M3/M3a/M3b/M3c/M3d` branch and, more importantly, it stays fully stable and contact-clean while the chair is freer than in the old ground-locked bridges.
91. The motion curriculum has now changed shape. The retained path is no longer “left-hard/right-soft bridge back into the old free-chair `M3` task.” The better path is:
    - retained `M2 model_9900.pt`
    - retained `M3e model_9950.pt` for clean both-hard constrained motion
    - retained `M3f model_10049.pt` for free-yaw heavy-damped both-hard forward motion
    The next useful lever is to start reducing the remaining heavy planar damping on `M3f`, not to return to the older soft-right motion family.
92. A first attempt to reduce that planar damping was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3g-FreeYawMediumDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3g` relaxed the retained `M3f` scaffold from heavy planar damping to medium planar damping (`y_velocity_scale = 0.25`, `yaw_velocity_scale = 0.25`) while also strengthening the lateral/line/yaw penalties. The bounded continuation from retained `M3f model_10049.pt` selected `model_10100.pt` with:
    `forward_motion_score = 0.37746714847162366`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.011438323184847832`,
    `wheelchair_lateral_velocity_abs_mean = 0.007954314351081848`,
    and `wheelchair_yaw_velocity_abs_mean = 0.026653502136468887`.
    This stayed fully stable and contact-clean, but it was still a slight regression from retained `M3f` on the primary motion score and a clear regression in lateral/yaw cleanliness, so `M3g` was discarded rather than promoted into the retained ladder.
93. The next probe isolated the damping change instead of changing both dynamics and reward shaping at once:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3h-FreeYawIntermediateDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3h` keeps the retained `M3f` reward scaffold exactly the same and only relaxes the planar damping to an intermediate step (`y_velocity_scale = 0.15`, `yaw_velocity_scale = 0.15`). Starting from retained `M3f model_10049.pt`, the bounded continuation selected `model_10148.pt` with:
    `forward_motion_score = 0.377978881332092`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.012244169600307941`,
    `wheelchair_lateral_velocity_abs_mean = 0.004626037552952766`,
    and `wheelchair_yaw_velocity_abs_mean = 0.015049846842885017`.
    This is still a small same-task regression from retained `M3f` on pure forward-motion score, but unlike `M3g` it stayed close to `M3f` while materially increasing chair freedom. Because it is fully stable, fully contact-clean, and substantially cleaner than `M3g`, `M3h` is retained as the next motion rung.
94. The retained motion ladder is now:
    - retained `M2 model_9900.pt`
    - retained `M3e model_9950.pt`
    - retained `M3f model_10049.pt`
    - retained `M3h model_10148.pt`
    The next useful lever is no longer another broad damping jump. It should be either a downstream test from `M3h` into a lighter rung or another narrowly isolated release of the planar damping, using `M3h` rather than `M3f` as the source.
95. That next isolated release was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3i-FreeYawMediumDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3i` starts from retained `M3h` and keeps the same reward scaffold, the same observations, and the same both-hard grip. The only change is another planar-damping release to `y_velocity_scale = 0.25` and `yaw_velocity_scale = 0.25`. Starting from retained `M3h model_10148.pt`, the bounded continuation selected `model_10200.pt` with:
    `forward_motion_score = 0.3813771064276807`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.012850762344896793`,
    `wheelchair_lateral_velocity_abs_mean = 0.007191838696599007`,
    and `wheelchair_yaw_velocity_abs_mean = 0.02351665124297142`.
    This is a real improvement over retained `M3h` on the primary motion score while preserving full stability and zero invalid contact. It is still somewhat less laterally clean than retained `M3f`, but because the chair is now freer and the policy remained physically valid, `M3i` is retained as the next rung.
96. The retained motion ladder is now:
    - retained `M2 model_9900.pt`
    - retained `M3e model_9950.pt`
    - retained `M3f model_10049.pt`
    - retained `M3h model_10148.pt`
    - retained `M3i model_10200.pt`
    The next useful experiment is to keep the same observation/reward scaffold again and either test a still lighter free-yaw damping rung from `M3i`, or start introducing explicit backward/turn command structure on top of this cleaner forward-motion ladder.
97. That first lighter free-yaw damping jump from `M3i` was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3j-FreeYawLightBridgeDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3j` kept the retained `M3i` scaffold unchanged and only relaxed the planar damping further to `y_velocity_scale = 0.40` and `yaw_velocity_scale = 0.40`. Starting from retained `M3i model_10200.pt`, the bounded continuation selected `model_10200.pt` with:
    `forward_motion_score = 0.37235095321666456`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.008854018524289131`,
    `wheelchair_lateral_velocity_abs_mean = 0.012296464294195175`,
    and `wheelchair_yaw_velocity_abs_mean = 0.039986491203308105`.
    This stayed physically valid, but it regressed from retained `M3i` on the primary motion score, forward velocity, and lateral/yaw cleanliness, so `M3j` was discarded rather than promoted into the retained ladder.
98. The finer bridge between retained `M3i` and discarded `M3j` was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3k-FreeYawTransitionDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3k` kept the retained `M3i` reward and observation scaffold unchanged and only relaxed the planar damping partway to the failed `M3j` jump (`y_velocity_scale = 0.325`, `yaw_velocity_scale = 0.325`). Starting from retained `M3i model_10200.pt`, the bounded continuation selected `model_10299.pt` with:
    `forward_motion_score = 0.3819064769661054`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.00948390457779169`,
    `wheelchair_lateral_velocity_abs_mean = 0.009322939440608025`,
    and `wheelchair_yaw_velocity_abs_mean = 0.030022375285625458`.
    Relative to retained `M3i`, this is only a narrow primary-score improvement (`0.3819064769661054` vs `0.3813771064276807`) and it gives up some forward-velocity and yaw/lateral cleanliness. But it does so while keeping the chair freer than `M3i`, and it remains fully stable and fully contact-clean, so `M3k` is retained as the next bridge rung rather than discarded.
99. The retained forward-motion ladder is now:
    - retained `M2 model_9900.pt`
    - retained `M3e model_9950.pt`
    - retained `M3f model_10049.pt`
    - retained `M3h model_10148.pt`
    - retained `M3i model_10200.pt`
    - retained `M3k model_10299.pt`
    The next useful lever is no longer another big damping jump. It should be either one more small free-yaw damping release from `M3k`, or the first explicit backward/turn command branch on top of this now-cleaner forward ladder.
100. That next smaller free-yaw damping release from retained `M3k` was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3l-FreeYawLightTransitionDampedForward-Observed-BothHard-RelaxedHandle`.
    `M3l` kept the retained `M3k` reward and observation scaffold unchanged and only relaxed the planar damping again to `y_velocity_scale = 0.35` and `yaw_velocity_scale = 0.35`. Starting from retained `M3k model_10299.pt`, the bounded continuation selected `model_10398.pt` with:
    `forward_motion_score = 0.38274107103934507`, `clean_hold_rate = 1.0`, `time_out_rate = 1.0`, `bad_orientation_rate = 0.0`, `invalid_contact_rate = 0.0`,
    `wheelchair_forward_velocity_mean = 0.007933719083666801`,
    `wheelchair_lateral_velocity_abs_mean = 0.009734446182847023`,
    and `wheelchair_yaw_velocity_abs_mean = 0.03142866492271423`.
    Relative to retained `M3k`, this is again only a narrow primary-score improvement (`0.38274107103934507` vs `0.3819064769661054`). It gives up some raw forward velocity and a bit of yaw/lateral cleanliness, but it does so at still-freer chair dynamics while staying fully stable and fully contact-clean. So `M3l` is retained as the next bridge rung, with the caveat that this branch is now clearly in diminishing-return territory.
101. The retained forward-motion ladder is now:
    - retained `M2 model_9900.pt`
    - retained `M3e model_9950.pt`
    - retained `M3f model_10049.pt`
    - retained `M3h model_10148.pt`
    - retained `M3i model_10200.pt`
    - retained `M3k model_10299.pt`
    - retained `M3l model_10398.pt`
    The next useful branch should stop treating freer forward-only damping release as the only lever. The cleaner next move is either the first explicit backward/turn command-conditioned stage on top of `M3l`, or a motion-stage redesign that rewards more actual chair speed instead of marginal score gains from trading forward speed against lateral/yaw behavior.
102. The evaluator now exposes a command-aligned linear motion metric, `command_motion_score`, for signed wheelchair command tasks. It keeps the same stability/contact penalties as `forward_motion_score`, but replaces the raw forward-velocity term with a command-aligned ratio so a physically good backward policy is not scored as failure just because its wheelchair velocity is negative in world X.
103. The first explicit backward branch from retained `M3l` was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4a-FreeYawLightTransitionDampedBackward-Observed-BothHard-RelaxedHandle`.
    `M4a` keeps the retained `M3l` free-yaw damping scaffold, flips the commanded X velocity to `-1.0`, zeroes the forward-progress reward, switches `wheelchair_backward_velocity` to the linear form, and biases the robot lean target slightly backward. Zero-shot transfer from retained `M3l model_10398.pt` already produced real backward chair motion, but through obvious failure modes: `command_motion_score = 0.25261443648487325`, `clean_hold_rate = 0.03125`, `time_out_rate = 0.0546875`, `bad_orientation_rate = 0.8984375`, `invalid_contact_rate = 0.890625`.
104. A bounded continuation on `M4a` selected `model_10497.pt` with:
    `command_motion_score = 0.2613088373094797`, `clean_hold_rate = 0.1015625`, `time_out_rate = 0.125`, `bad_orientation_rate = 0.84375`, `invalid_contact_rate = 0.75`,
    `wheelchair_command_aligned_velocity_ratio = 0.4343257546424866`,
    `wheelchair_forward_velocity_mean = -0.4343257546424866`,
    `wheelchair_lateral_velocity_abs_mean = 0.0511283352971077`,
    and `wheelchair_yaw_velocity_abs_mean = 0.1161910742521286`.
    This confirms that backward chair motion transfers directionally from the forward ladder, but the branch is still not physically usable. The continuation improves survival and reduces invalid contact compared with zero-shot transfer, yet it still relies heavily on bad orientation and chair-body contact. So `M4a` is not retained as a milestone; it is the first diagnostic backward branch, and the next backward iteration needs stronger stability/contact shaping rather than more damping release.
105. A stricter signed-motion metric was then added:
    `physical_command_motion_score`.
    Unlike the earlier `command_motion_score`, it weights `clean_hold_rate`, `time_out_rate`, `bad_orientation_rate`, `base_height_rate`, and `invalid_contact_rate` much more heavily. The old scalar was too generous for backward pulling: it could score a fast but physically bad rollout as progress simply because the wheelchair moved backward at the commanded speed.
106. The next backward branch was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4b-FreeYawLightTransitionDampedBackward-Stabilized-Observed-BothHard-RelaxedHandle`.
    `M4b` removed the duplicated raw backward-speed reward, kept the signed command-tracking term as the only dense backward objective, and restored light posture/contact shaping (`flat_orientation_l2`, `base_height`, `robot_xy_velocity`, `robot_yaw_velocity`, stronger `wheelchair_invalid_contact`). On a fair `64 env / 300 step` deterministic eval, the best saved checkpoint was `model_10400.pt` with:
    `physical_command_motion_score = -0.11300523318350317`,
    `clean_hold_rate = 0.0`,
    `time_out_rate = 0.0`,
    `bad_orientation_rate = 0.578125`,
    `base_height_rate = 0.03125`,
    `invalid_contact_rate = 0.484375`,
    `wheelchair_command_aligned_velocity_ratio = 0.5938286781311035`.
    The useful diagnostic is where the contact lives:
    `wheelchair_base_robot_contact = 658.8596`,
    `wheelchair_left_rear_wheel_robot_contact = 255.9626`,
    `wheelchair_right_rear_wheel_robot_contact = 21.5525`,
    while both handle invalid-contact sensors stayed at `0.0`.
107. The fair `64 env / 300 step` comparison against the original `M4a model_10497.pt` showed that `M4b` did not actually beat it as a branch. `M4a model_10497.pt` scored
    `physical_command_motion_score = -0.10337315350770951`
    with the same `clean_hold_rate = 0.0` and `time_out_rate = 0.0`.
    `M4b` did reduce base-height failures and removed the small left-handle invalid-contact leak, but it did not solve the real blocker. The dominant failure mode is still torso/chair-base plus left-rear-wheel contact while the robot collapses backward into the chair.
108. A second probe moved the backward task earlier in the dynamics ladder:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4c-FreeYawHeavyDampedBackward-Stabilized-Observed-BothHard-RelaxedHandle`.
    Zero-shot transfer from retained `M3f model_10049.pt` came back worse:
    `physical_command_motion_score = -0.16158021707087755`,
    `bad_orientation_rate = 0.796875`,
    `invalid_contact_rate = 0.625`,
    with the same dominant invalid-contact pattern (`wheelchair_base_robot_contact` plus `wheelchair_left_rear_wheel_robot_contact`).
    So stepping earlier to heavier free-yaw damping did not fix backward pulling either.
109. The pre-`M4d` backward read after `M4a/M4b/M4c` was:
    backward chair motion transferred directionally from the retained forward ladder, but there was still no retained physically valid backward milestone. The meaningful lesson from `M4a/M4b/M4c` was that the blocker was not missing sign information in the reward, and it was not primarily handle contact. The blocker was geometric/postural collapse into the chair base and left rear wheel during pullback. So the next backward branch had to target that specific failure mode directly, likely with a changed backward manipulation scaffold or clearance/separation shaping rather than another raw speed reward change.
110. The first branch that directly targeted that failure mode was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4d-FreeYawHeavyDampedBackward-Creep-Observed-BothHard-RelaxedHandle`.
    `M4d` keeps the retained `M3f` heavy-damped both-hard scaffold, but changes the backward curriculum in three specific ways:
    - it slows the commanded backward speed down to a fixed creep target of `-0.14 m/s`
    - it removes the extra raw backward-speed shaping and keeps signed command tracking as the dense motion term
    - it adds explicit robot-frame chair-separation shaping through `wheelchair_robot_standoff`, penalizing drift of the wheelchair root away from its nominal XY offset in the robot root frame
    This is the first backward branch that became physically clean instead of collapsing into the chair. Zero-shot transfer from retained `M3f model_10049.pt` on a full `64 env / 600 step` deterministic eval produced:
    `physical_command_motion_score = 1.2724524709396063`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.7269237637519836`,
    `wheelchair_forward_velocity_mean = -0.13679799437522888`,
    `wheelchair_lateral_velocity_abs_mean = 0.003717011772096157`,
    and `wheelchair_yaw_velocity_abs_mean = 0.009016682393848896`.
    So the retained forward `M3f` checkpoint already transfers cleanly into slow backward creep when the task is eased enough and the chair-separation geometry is made explicit.
111. A bounded low-noise continuation on `M4d` from retained `M3f model_10049.pt` then wrote three checkpoints: `model_10050.pt`, `model_10100.pt`, and `model_10148.pt`. All three stayed fully clean on the same `64 env / 600 step` deterministic eval, and each slightly improved on the zero-shot baseline. The best saved checkpoint was `model_10148.pt` with:
    `physical_command_motion_score = 1.282831170875579`,
    `command_motion_score = 1.1443229076452552`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.7383108139038086`,
    `wheelchair_forward_velocity_mean = -0.1339568942785263`,
    `wheelchair_lateral_velocity_abs_mean = 0.003613825421780348`,
    and `wheelchair_yaw_velocity_abs_mean = 0.008656003512442112`.
    The gain over zero-shot is small, but it is real and consistent, so `M4d model_10148.pt` is retained as the first physically valid backward curriculum rung. It is not yet the final `M5` backward-control milestone, because the commanded speed is still only a slow creep and the dynamics are still the easier heavy-damped branch, but it is the first backward stage that is worth building on instead of discarding.

## Autoresearch Harness

The first `codex-autoresearch` loop targeted `M0`, not the later motion phases.

The original task was:

`Unitree-G1-29dof-Wheelchair-Scratch-M0-CollidableStand-DirectObs`

The retained `M0` task is now:

`Unitree-G1-29dof-Wheelchair-Scratch-M0-CollidableStand-Observed`

Mechanical verify command:

```bash
conda run --no-capture-output -n isaaclab python scripts/autoresearch/benchmark_wheelchair_m0.py --metric m0_score
```

That command lives in `unitree_rl_lab` and does two things: it runs a short bounded training continuation on the requested task, then it evaluates the resulting checkpoint with a deterministic rollout. The primary score is `m0_score`, but the evaluator also records `clean_hold_rate`, `invalid_contact_rate`, `bad_orientation_rate`, and `base_height_rate`. Phase advancement should not be decided from `m0_score` alone; it is only the dense optimization signal for the loop.

The default verifier intentionally omits the per-handle invalid-contact filter breakdown. That breakdown is still available as a diagnosis-only path in the evaluator, but it is not part of the unattended loop because it is materially heavier than the aggregate metric path.

For the unattended background loop, use the metrics-only JSON variant instead so the runtime can keep `m0_score` as the primary metric while also enforcing acceptance gates on `clean_hold_rate` and `invalid_contact_rate`:

```bash
conda run --no-capture-output -n isaaclab python scripts/autoresearch/benchmark_wheelchair_m0.py --metrics-json-only
```

For later bridge stages such as `M1f`, `M1g`, and `M2a`, the single-stage `M0` wrapper is not sufficient. Use
`scripts/autoresearch/benchmark_wheelchair_bridge.py`
instead so the loop can score both same-stage hold quality and downstream transfer into the next damping rung, with downstream `m0_score` used as the primary metric.

For motion-stage work such as `M3`, the same bridge harness should use `--primary-metric-key forward_motion_score`. That metric is now directional and should be treated as the gate: backward rail motion or near-zero chair motion is not a pass even if the rollout survives cleanly.
