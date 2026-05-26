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
112. The next backward rung was a pure command-difficulty increase on top of retained `M4d`:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4e-FreeYawHeavyDampedBackward-Moderate-Observed-BothHard-RelaxedHandle`.
    `M4e` keeps the same heavy-damped both-hard scaffold and the same explicit `wheelchair_robot_standoff` shaping, but increases the fixed commanded backward speed from `-0.14 m/s` to `-0.25 m/s` and slightly widens the command-tracking standard deviation to `0.10`. Zero-shot transfer from retained `M4d model_10148.pt` stayed physically very strong:
    `physical_command_motion_score = 1.106793893314898`,
    `clean_hold_rate = 0.984375`,
    `time_out_rate = 0.984375`,
    `bad_orientation_rate = 0.015625`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5650618672370911`,
    `wheelchair_forward_velocity_mean = -0.14822953939437866`,
    `wheelchair_lateral_velocity_abs_mean = 0.004035579971969128`,
    and `wheelchair_yaw_velocity_abs_mean = 0.010057952255010605`.
    So the harder backward command did not reopen the old chair-contact failure mode; it only cost a small amount of stability and tracking.
113. A short low-noise `M4e` continuation from retained `M4d model_10148.pt` then wrote two checkpoints: `model_10150.pt` and `model_10197.pt`. `model_10150.pt` regressed slightly by introducing a small base-contact leak (`invalid_contact_rate = 0.015625`), so it was not retained. The later checkpoint, `model_10197.pt`, recovered zero invalid contact and slightly improved the harder backward task over the zero-shot baseline:
    `physical_command_motion_score = 1.108896442782134`,
    `command_motion_score = 0.9297696615569295`,
    `clean_hold_rate = 0.984375`,
    `time_out_rate = 0.984375`,
    `bad_orientation_rate = 0.015625`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5706008672714233`,
    `wheelchair_forward_velocity_mean = -0.14852306246757507`,
    `wheelchair_lateral_velocity_abs_mean = 0.003994424361735582`,
    and `wheelchair_yaw_velocity_abs_mean = 0.010136200115084648`.
    This is still not the final backward-control milestone, because the branch remains slightly less stable than retained `M4d` and is still on the easier heavy-damped dynamics. But it is a valid retained curriculum rung: the policy stays contact-clean at the faster backward command and does not collapse back into the chair.
114. The retained backward ladder is now:
    - retained `M4d model_10148.pt` for physically clean backward creep
    - retained `M4e model_10197.pt` for the first faster backward pull on the same clean scaffold
    The next useful branch is still not “final backward control.” The right next lever is another command/dynamics increase from `M4e`, while preserving the chair-separation shaping that removed the old base and rear-wheel collapse.
115. That next branch released dynamics instead of increasing the backward command again:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4f-FreeYawIntermediateDampedBackward-Moderate-Observed-BothHard-RelaxedHandle`.
    `M4f` keeps the retained `M4e` backward command at `-0.25 m/s`, keeps the same both-hard grip and the same explicit `wheelchair_robot_standoff` shaping, and only relaxes the planar damping from the heavy branch to the retained forward `M3h` level (`y_velocity_scale = 0.15`, `yaw_velocity_scale = 0.15`). Zero-shot transfer from retained `M4e model_10197.pt` was already strong and fully clean:
    `physical_command_motion_score = 1.1096096923574805`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5538078546524048`,
    `wheelchair_forward_velocity_mean = -0.1447066366672516`,
    `wheelchair_lateral_velocity_abs_mean = 0.012959773652255535`,
    and `wheelchair_yaw_velocity_abs_mean = 0.03162518888711929`.
    The important part is not the raw score bump. It is that the backward policy stayed fully stable and fully contact-clean after the first real damping release.
116. A short low-noise continuation on `M4f` from retained `M4e model_10197.pt` then wrote two checkpoints: `model_10200.pt` and `model_10226.pt`. Both stayed fully clean, but `model_10200.pt` was slightly better:
    `physical_command_motion_score = 1.1096278823912142`,
    `command_motion_score = 0.9126209668815137`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5522634387016296`,
    `wheelchair_forward_velocity_mean = -0.14592473208904266`,
    `wheelchair_lateral_velocity_abs_mean = 0.012861572206020355`,
    and `wheelchair_yaw_velocity_abs_mean = 0.03229823708534241`.
    The gain over zero-shot is extremely small, but it remains a valid retained rung because it preserves full physical cleanliness at the lighter damping level.
117. The retained backward ladder is now:
    - retained `M4d model_10148.pt` for physically clean backward creep
    - retained `M4e model_10197.pt` for the first faster backward pull on heavy damping
    - retained `M4f model_10200.pt` for the same `-0.25 m/s` backward task after the first damping release
    The next useful branch is to keep the command fixed and continue releasing dynamics one rung at a time before asking for a still faster backward pull.
118. The next backward rung was that exact dynamics-only release:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4g-FreeYawMediumDampedBackward-Moderate-Observed-BothHard-RelaxedHandle`.
    `M4g` keeps the retained `M4f` command at `-0.25 m/s`, keeps the same both-hard grip and the same `wheelchair_robot_standoff` shaping, and only relaxes planar damping one more step to the retained forward `M3i` level (`y_velocity_scale = 0.25`, `yaw_velocity_scale = 0.25`). Zero-shot transfer from retained `M4f model_10200.pt` was already physically valid:
    `physical_command_motion_score = 1.0627775263041257`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5247573852539062`,
    `wheelchair_forward_velocity_mean = -0.1451890766620636`,
    `wheelchair_lateral_velocity_abs_mean = 0.023683326318860054`,
    and `wheelchair_yaw_velocity_abs_mean = 0.056736476719379425`.
    So `M4g` is a valid rung even before continuation, but it gives up some yaw/lateral cleanliness compared with retained `M4f`.
119. A short low-noise continuation on `M4g` from retained `M4f model_10200.pt` then wrote two checkpoints: `model_10200.pt` and `model_10229.pt`. Both stayed fully stable and fully contact-clean. The later checkpoint was slightly better and is retained:
    `physical_command_motion_score = 1.0778401739895345`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.536535382270813`,
    `wheelchair_forward_velocity_mean = -0.14621871709823608`,
    `wheelchair_lateral_velocity_abs_mean = 0.022800607606768608`,
    and `wheelchair_yaw_velocity_abs_mean = 0.05446473881602287`.
    The score gain over the zero-shot `M4g` transfer is small, but it is real, and the branch stays fully physically valid at the freer damping level. The retained backward ladder is now:
    - retained `M4d model_10148.pt` for physically clean backward creep
    - retained `M4e model_10197.pt` for the faster heavy-damped backward pull
    - retained `M4f model_10200.pt` for the first damping release at `-0.25 m/s`
    - retained `M4g model_10229.pt` for the next medium-damped backward rung on the same clean scaffold
    The next useful move is the same pattern again: keep the backward command fixed and release dynamics one more rung before increasing backward speed.
120. The next backward rung was that next dynamics-only release:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4h-FreeYawTransitionDampedBackward-Moderate-Observed-BothHard-RelaxedHandle`.
    `M4h` keeps the retained `M4g` command at `-0.25 m/s`, keeps the same both-hard grip and the same `wheelchair_robot_standoff` shaping, and only relaxes planar damping one more step to the retained forward `M3k` level (`y_velocity_scale = 0.325`, `yaw_velocity_scale = 0.325`). Zero-shot transfer from retained `M4g model_10229.pt` stayed fully physically valid:
    `physical_command_motion_score = 1.0293966956436635`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5074198842048645`,
    `wheelchair_forward_velocity_mean = -0.14734360575675964`,
    `wheelchair_lateral_velocity_abs_mean = 0.03193458169698715`,
    and `wheelchair_yaw_velocity_abs_mean = 0.07378971576690674`.
    This is a small same-task regression versus retained `M4g`, but it is still a real physically valid backward rung at freer chair dynamics.
121. A short low-noise continuation on `M4h` from retained `M4g model_10229.pt` then wrote two checkpoints: `model_10250.pt` and `model_10258.pt`. Both stayed fully stable and fully contact-clean. The later checkpoint was slightly better and is retained:
    `physical_command_motion_score = 1.0570705771446227`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5301052927970886`,
    `wheelchair_forward_velocity_mean = -0.14599697291851044`,
    `wheelchair_lateral_velocity_abs_mean = 0.03068336471915245`,
    and `wheelchair_yaw_velocity_abs_mean = 0.07100684940814972`.
    `M4h` still does not beat retained `M4g` on the primary same-task score, but it recovers part of the zero-shot drop and preserves full physical cleanliness at the freer damping level. The retained backward ladder is now:
    - retained `M4d model_10148.pt` for physically clean backward creep
    - retained `M4e model_10197.pt` for the faster heavy-damped backward pull
    - retained `M4f model_10200.pt` for the first damping release at `-0.25 m/s`
    - retained `M4g model_10229.pt` for the medium-damped backward rung
    - retained `M4h model_10258.pt` for the next transition-damped backward rung at the same command
    The next useful move is the same one again: keep the backward command fixed and release dynamics one more rung before increasing backward speed or mixing in turning commands.
122. The next backward rung was the light-transition release to the retained forward `M3l` damping level:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4i-FreeYawLightTransitionDampedBackward-Moderate-Observed-BothHard-RelaxedHandle`.
    `M4i` keeps the retained `M4h` command at `-0.25 m/s`, keeps the same both-hard grip and the same `wheelchair_robot_standoff` shaping, and only relaxes planar damping one more step to `y_velocity_scale = 0.35` and `yaw_velocity_scale = 0.35`. Zero-shot transfer from retained `M4h model_10258.pt` exposed the first backward contact leak on this ladder:
    `physical_command_motion_score = 1.039323188364506`,
    `clean_hold_rate = 0.984375`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.015625`,
    with the entire leak concentrated in `wheelchair_base_robot_contact`.
    So `M4i` was initially beyond the clean backward boundary, but only slightly.
123. A short low-noise continuation on `M4i` from retained `M4h model_10258.pt` then recovered that leak cleanly. The selected checkpoint `model_10287.pt` came back at:
    `physical_command_motion_score = 1.0621339753270151`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_backward_velocity_ratio = 0.5296800136566162`,
    `wheelchair_forward_velocity_mean = -0.14203692972660065`,
    `wheelchair_lateral_velocity_abs_mean = 0.031298667192459106`,
    and `wheelchair_yaw_velocity_abs_mean = 0.07465916872024536`.
    That makes `M4i` a valid retained rung after continuation. It is slightly freer than retained `M4h`, fully stable, and fully contact-clean again. The retained backward ladder is now:
    - retained `M4d model_10148.pt` for physically clean backward creep
    - retained `M4e model_10197.pt` for the faster heavy-damped backward pull
    - retained `M4f model_10200.pt` for the first damping release at `-0.25 m/s`
    - retained `M4g model_10229.pt` for the medium-damped backward rung
    - retained `M4h model_10258.pt` for the transition-damped backward rung
    - retained `M4i model_10287.pt` for the light-transition backward rung after contact recovery
    The next useful move is to decide whether the backward ladder can absorb one final dynamics release cleanly, or whether this is the point to stop releasing damping and start mixing in turn command structure.

124. The first turn branch started from the retained free-yaw both-hard motion scaffold:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6a-FreeYawHeavyDampedLeftTurn-Observed-BothHard-RelaxedHandle`.
    `M6a` keeps the retained `M3f` chair dynamics, sets `lin_vel_x = 0`, `lin_vel_y = 0`, `ang_vel_z = +0.35`, enables the new wheelchair yaw-tracking reward, and suppresses forward-motion-specific shaping. Zero-shot transfer from retained `M3f model_10049.pt` was already a valid left-turn milestone with the corrected turn gate:
    `physical_turn_motion_score = 0.6742888854518624`,
    `turn_motion_score = 1.3318987463135272`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.887657642364502`,
    `wheelchair_command_aligned_yaw_ratio_clipped = 0.8880758881568909`,
    `wheelchair_yaw_tracking_mean = 0.3497166037559509`,
    and `wheelchair_yaw_velocity_mean = 0.5226268768310547`.
    So `M6a` is retained immediately. Left turning exists on the current ladder.

125. The mirrored right-turn task was then added as
    `Unitree-G1-29dof-Wheelchair-Scratch-M6b-FreeYawHeavyDampedRightTurn-Observed-BothHard-RelaxedHandle`.
    `M6b` is intentionally the exact mirror of `M6a` except for the command sign: `ang_vel_z = -0.35`. This branch exposed a real asymmetry. Zero-shot transfer from retained `M3f model_10049.pt` stayed physically clean, but the chair still turned left:
    `wheelchair_yaw_velocity_mean = 0.35613349080085754`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.6341801881790161`,
    and `turn_motion_score = -0.29364724527113134`.
    A short low-noise continuation from the same source improved rollout cleanliness but not signed turning. The best checkpoint from that run, `model_10050.pt`, still failed the actual task:
    `physical_turn_motion_score = 0.0456698986813967`,
    `turn_motion_score = -0.22634734716266397`,
    `clean_hold_rate = 0.96875`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.5703282356262207`,
    and `wheelchair_yaw_velocity_mean = 0.32910603284835815`.
    So `M6b` is not retained as a right-turn milestone. It is a stable wrong-sign turn.

126. That wrong-sign behavior is not just a bad warm start from one checkpoint. The same right-turn task was probed from the cleaner motion and hold sources as well:
    - retained `M3e model_9950.pt`: `wheelchair_command_aligned_yaw_ratio_symmetric = -0.6749926209449768`
    - retained `M4i model_10287.pt`: `wheelchair_command_aligned_yaw_ratio_symmetric = -0.571495771408081`
    - retained `M2 model_9900.pt`: `wheelchair_command_aligned_yaw_ratio_symmetric = -0.6318390965461731`
    All of them stayed mostly or fully stable and contact-clean, and all of them still turned left under the negative yaw command. That means the immediate blocker for right turning is not simply a bad source checkpoint. The current both-hard heavy-damped turn scaffold is itself left-biased under the mirrored command.

127. The turn evaluator had to be corrected before retaining any right-turn result. The original `physical_turn_motion_score` could still stay moderately positive when the rollout was stable and contact-clean even if the chair turned the wrong way. The corrected gate now multiplies the physical turn base score by `wheelchair_command_aligned_yaw_ratio_clipped`, so wrong-sign yaw cannot win selection just by surviving cleanly. This matters for future automation too. The wheelchair observed policy already includes signed `velocity_commands`, so the right-turn failure is not explained by a missing yaw command observation. The next right-turn branch should change the turn scaffold itself, most likely by changing grip asymmetry or handle-assist structure, not by blindly running more of the current mirrored both-hard task.

128. That next scaffold change was exactly the missing mirrored asymmetry:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6c-FreeYawHeavyDampedRightTurn-Observed-RightHardLeftSoft-RelaxedHandle`.
    `M6c` keeps the same heavy-damped right-turn command as `M6b`, but replaces the failing both-hard grip with the mirror of the successful left-turn asymmetry: the right hand stays as the hard attachment and the left hand becomes a bounded soft assist with mirrored handle-position and axis-alignment shaping. This immediately fixed the sign problem. Zero-shot transfer from retained `M3f model_10049.pt` came back fully stable, fully contact-clean, and directionally correct:
    `physical_turn_motion_score = 0.6617681152270161`,
    `turn_motion_score = 1.322267762525007`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `bad_orientation_rate = 0.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.712121307849884`,
    `wheelchair_command_aligned_yaw_ratio_clipped = 0.7412710785865784`,
    `wheelchair_yaw_tracking_mean = 0.6739866733551025`,
    and `wheelchair_yaw_velocity_mean = -0.27772974967956543`.
    So `M6c` is retained immediately as the first valid right-turn milestone.

129. The current turn read is now much cleaner:
    - retained `M6a` for left turn on the both-hard heavy-damped scaffold
    - discarded `M6b` because the mirrored both-hard right-turn scaffold stayed wrong-sign
    - retained `M6c` because the mirrored right-hard/left-soft scaffold fixes the sign without losing stability
    The important lesson is that turning on this wheelchair is not symmetric under the both-hard grip. Left turn works on the symmetric scaffold, but right turn needs the mirrored asymmetric grip structure. That is now concrete evidence, not speculation.

130. The next question was whether that retained `M6c` right-hard/left-soft scaffold could serve as a single fixed grip structure for a future unified controller. Three direct probes answered that.
    The first was a forward probe on the same heavy-damped motion stage:
    `Unitree-G1-29dof-Wheelchair-Scratch-M3m-FreeYawHeavyDampedForward-Observed-RightHardLeftSoft-RelaxedHandle`.
    Zero-shot transfer from retained `M3f model_10049.pt` stayed upright, but it was not a valid forward milestone:
    `physical_command_motion_score = 0.6925527523155324`,
    `clean_hold_rate = 0.921875`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.078125`,
    with the entire leak concentrated in `wheelchair_left_handle_invalid_contact`,
    and `wheelchair_forward_velocity_ratio = 0.05961471050977707`.
    So the fixed right-hard/left-soft scaffold is poor for forward propulsion.

131. The second probe was a left-turn task on the same fixed right-hard/left-soft scaffold:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6d-FreeYawHeavyDampedLeftTurn-Observed-RightHardLeftSoft-RelaxedHandle`.
    Zero-shot transfer from retained `M3f model_10049.pt` stayed physically clean, but it flipped the sign just like the earlier failed right-turn both-hard branch:
    `physical_turn_motion_score = 0.018247194337265026`,
    `turn_motion_score = -0.32261592620052393`,
    `clean_hold_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.7156335711479187`,
    and `wheelchair_yaw_velocity_mean = -0.28035983443260193`.
    So the fixed right-hard/left-soft scaffold is not a valid left-turn scaffold either.

132. The third probe was a backward task on that same fixed right-hard/left-soft scaffold:
    `Unitree-G1-29dof-Wheelchair-Scratch-M4j-FreeYawHeavyDampedBackward-Moderate-Observed-RightHardLeftSoft-RelaxedHandle`.
    This one did work. Zero-shot transfer from retained `M4i model_10287.pt` came back fully stable and contact-clean:
    `physical_command_motion_score = 1.315110251214355`,
    `command_motion_score = 1.1925020365975796`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_velocity_ratio_symmetric = 0.8068245649337769`.
    So the fixed right-hard/left-soft scaffold is strong for backward pulling, just like it is for right turning.

133. That gives a clean scaffold split:
    - both-hard heavy-damped is a strong retained scaffold for forward and left turn
    - right-hard/left-soft heavy-damped is a strong retained scaffold for backward and right turn
    - no single fixed grip structure tested so far supports all four commands cleanly
    This means the next unified-controller branch should not assume one fixed hard/soft asymmetry. The most defensible next scaffold is either:
    1. a command-conditioned attachment/stiffness scaffold that changes dominance with the command sign, or
    2. a new neutral two-soft-hands scaffold that is explicitly tested across all four commands before mixed-command training starts.

134. That neutral two-soft-hands branch was then tested directly on the same heavy-damped motion stage. Four zero-shot probes answered it cleanly.
    The forward probe was
    `Unitree-G1-29dof-Wheelchair-Scratch-M3n-FreeYawHeavyDampedForward-Observed-BothSoft-RelaxedHandle`.
    Zero-shot transfer from retained `M3f model_10049.pt` stayed fully stable and fully contact-clean:
    `physical_command_motion_score = 0.9078378646518104`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_forward_velocity_ratio = 0.3246929943561554`.
    But it also carried a very large lateral-offset bias:
    `wheelchair_lateral_offset_norm = 0.9989199042320251`.
    So the neutral scaffold can translate forward without falling apart, but it is not especially clean.

135. The backward probe was
    `Unitree-G1-29dof-Wheelchair-Scratch-M4k-FreeYawHeavyDampedBackward-Moderate-Observed-BothSoft-RelaxedHandle`.
    Zero-shot transfer from retained `M4i model_10287.pt` was the strongest result on this neutral scaffold:
    `physical_command_motion_score = 1.3269853260484525`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_velocity_ratio_symmetric = 0.6706080436706543`.
    So the neutral scaffold is fully viable for backward pulling.

136. The turning probes were the real discriminator.
    Left turn on
    `Unitree-G1-29dof-Wheelchair-Scratch-M6e-FreeYawHeavyDampedLeftTurn-Observed-BothSoft-RelaxedHandle`
    stayed fully stable and contact-clean, but it produced almost no yaw authority:
    `physical_turn_motion_score = 0.005771564438546955`,
    `clean_hold_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.007619310170412064`,
    and `wheelchair_yaw_velocity_mean = 0.0026667648926377296`.
    Right turn on
    `Unitree-G1-29dof-Wheelchair-Scratch-M6f-FreeYawHeavyDampedRightTurn-Observed-BothSoft-RelaxedHandle`
    failed the same way:
    `physical_turn_motion_score = 0.003245790785507999`,
    `clean_hold_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.003883844008669257`,
    and `wheelchair_yaw_velocity_mean = -0.001359348651021719`.
    So the neutral two-soft scaffold is not a viable fixed turning scaffold in either direction. It preserves stability by effectively refusing to generate yaw.

137. That resolves the neutral-scaffold question.
    - both-soft heavy-damped can support translation, especially backward
    - both-soft heavy-damped does not provide usable left-turn or right-turn control
    - the fixed neutral scaffold therefore does not solve the unified-controller problem
    The next unified branch should not keep adding fixed grip variants. It should move to a command-conditioned attachment or stiffness scaffold that changes left/right dominance with the requested motion.

138. A narrower follow-up tested exactly that next idea, but only on turning before widening it into a full mixed-command scaffold. Two heavy-damped zero-shot probes used bounded soft attachments on both hands, with the left/right spring gains selected from the commanded yaw sign at runtime:
    - `Unitree-G1-29dof-Wheelchair-Scratch-M6g-FreeYawHeavyDampedLeftTurn-Observed-CommandConditionedSoft-RelaxedHandle`
    - `Unitree-G1-29dof-Wheelchair-Scratch-M6h-FreeYawHeavyDampedRightTurn-Observed-CommandConditionedSoft-RelaxedHandle`
    The left-turn probe stayed almost fully stable, but it still produced essentially no yaw:
    `physical_turn_motion_score = 0.006829837649564176`,
    `clean_hold_rate = 0.984375`,
    `invalid_contact_rate = 0.015625`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.005293993279337883`,
    and `wheelchair_yaw_velocity_mean = 0.0018529020017012954`.
    The tiny contact leak was concentrated entirely in `wheelchair_base_robot_contact`.
    The right-turn probe stayed fully stable and fully contact-clean, but it failed the same way:
    `physical_turn_motion_score = 0.00034842319505137`,
    `clean_hold_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.00770591339096427`,
    and `wheelchair_yaw_velocity_mean = 0.002697076415643096`.
    So command-conditioned stiffness on an all-soft grip is still too weak. It preserves stability, but it does not create usable turning authority.

139. That means the next unified-controller branch should skip further soft-only variants. The likely next mechanism is a command-conditioned hybrid scaffold that changes the attachment class itself, not just the spring gain: for example a dominant hard or spherical attachment on the commanded turn side plus a bounded assist on the opposite hand. The turning blocker is no longer “which side should dominate”; it is that soft-only dominance does not transmit enough yaw authority into the chair.

140. The remaining open question on turning was whether the successful right-turn asymmetry was just incidental to that one task family, or whether the hard/soft attachment class itself really determines the turn sign. Two direct probes on the existing left-hard/right-soft scaffold answered that cleanly:
    - `Unitree-G1-29dof-Wheelchair-Scratch-M6i-FreeYawHeavyDampedLeftTurn-Observed-LeftHardRightSoft-RelaxedHandle`
    - `Unitree-G1-29dof-Wheelchair-Scratch-M6j-FreeYawHeavyDampedRightTurn-Observed-LeftHardRightSoft-RelaxedHandle`
    On `M6i`, zero-shot transfer from retained `M3f model_10049.pt` was fully stable and fully contact-clean, and it produced a very strong correct-sign left turn:
    `physical_turn_motion_score = 0.7484071274287998`,
    `turn_motion_score = 1.4161572684533892`,
    `clean_hold_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = 1.0`,
    and `wheelchair_yaw_velocity_mean = 0.8289926052093506`.
    On `M6j`, the exact same scaffold flipped back to the wrong sign under the mirrored right-turn command:
    `physical_turn_motion_score = 0.0`,
    `turn_motion_score = -0.6045880594581831`,
    `clean_hold_rate = 0.984375`,
    `invalid_contact_rate = 0.0`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.9685173630714417`,
    and `wheelchair_yaw_velocity_mean = 0.4204327166080475`.

141. That completes the asymmetric turn table:
    - both-hard heavy-damped: strong left turn, wrong-sign right turn
    - right-hard/left-soft heavy-damped: strong right turn, wrong-sign left turn
    - left-hard/right-soft heavy-damped: strong left turn, wrong-sign right turn
    - soft-only heavy-damped: stable but essentially no turn in either direction
    So the next unified-controller branch is no longer ambiguous. It should be a command-conditioned hybrid attachment policy that switches the dominant hard/soft side with the turn command sign. The turn problem is not just reward shaping; it is attachment topology.

142. A first direct attempt to encode that mixed topology inside one task did not fail as a policy result; it failed mechanically during environment bring-up and was removed. The discarded task was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6k-FreeYawHeavyDampedMixedTurn-Observed-SplitHybrid-RelaxedHandle`.
    The design was:
    - even environment ids: `left-hard/right-soft`
    - odd environment ids: `right-hard/left-soft`
    - command sign split by environment parity so both left and right turn commands existed in one batch
    The reason for the env-parity split was an Isaac Lab reset-order constraint: reset-mode events run before the command manager resamples commands, so the hard attachment side cannot be switched from the freshly sampled yaw sign in the ordinary reset event path.
    In practice, `M6k` never reached deterministic evaluation. On repeated runs at `1`, `2`, `16`, and `64` environments, the process exited during environment setup before the manager/observation summary and before any `AUTORESEARCH_METRIC` or output file was written. The same evaluator and source checkpoint completed normally on the neighboring fixed task `M6a`, so this was not a shared evaluator failure. The last emitted `M6k` log lines were the base-environment setup banner plus the standard `/World/envs/.../Robot` rigid-body-property warnings, then silent exit.
    So `M6k` is discarded as a task-construction/runtime failure, not as evidence about mixed-command policy quality. The lesson is narrower: the current env-parity mixed-turn scaffold is not robust inside the present Isaac Lab manager/event path. The next unified-controller branch should stay inside the mechanically proven task family and use a different command-conditioned hybrid mechanism instead of parity-split hard/soft reset events.

143. The next unified-turn probe stayed entirely inside the mechanically proven both-hard heavy-damped scaffold and only mixed the command sign:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6l-FreeYawHeavyDampedMixedTurn-Observed-BothHard-RelaxedHandle`.
    This is the first clean fixed-scaffold mixed left/right turn task. Zero-shot transfer from retained `M3f model_10049.pt` was fully stable and fully contact-clean:
    `physical_turn_motion_score = 0.3606618200187657`,
    `turn_motion_score = 0.5175286232959478`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_yaw_ratio_symmetric = 0.10960268974304199`.
    So the both-hard scaffold does not collapse under mixed-sign yaw commands. The failure mode is narrower: it still carries a strong left-turn bias, but it is no longer a pure wrong-sign branch.

144. A matched `50`-iteration model-only continuation on `M6l` from the same retained `M3f model_10049.pt` produced a small real improvement and is worth retaining as the current mixed-sign baseline:
    run:
    `unitree_g1_29dof_wheelchair_scratch_m6l_freeyaw_heavydamped_mixedturn_observed_both_hard_relaxedhandle/2026-05-26_13-05-14_mixedturn_bothhard_from_m3f10049_modelonly_50it`
    retained checkpoint:
    `model_10098.pt`
    deterministic eval:
    `physical_turn_motion_score = 0.3666192910436557`,
    `turn_motion_score = 0.5391623704694211`,
    `clean_hold_rate = 1.0`,
    `time_out_rate = 1.0`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_yaw_ratio_symmetric = 0.12723684310913086`.
    This is still far from a good unified turn controller, but it is the first retained mixed-sign turn rung that stays fully stable and fully contact-clean while showing nonzero bidirectional command alignment on one fixed attachment topology.

145. A follow-up reward-shaped branch tried to improve that same mixed-sign task without changing the scaffold itself. The discarded task was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6m-FreeYawHeavyDampedMixedTurn-Observed-BothHard-CommandShape-RelaxedHandle`.
    It kept the same both-hard heavy-damped runtime scaffold as `M6l`, and only changed training rewards so that the hand-handle pose and axis-alignment shaping followed the commanded yaw sign. As expected, zero-shot evaluation was identical to `M6l`, because the runtime policy weights were unchanged. The meaningful check was the same matched `50`-iteration model-only continuation from `M3f model_10049.pt`:
    run:
    `unitree_g1_29dof_wheelchair_scratch_m6m_freeyaw_heavydamped_mixedturn_observed_both_hard_command_shape_relaxedhandle/2026-05-26_13-11-32_mixedturn_bothhard_commandshape_from_m3f10049_modelonly_50it`
    evaluated checkpoint:
    `model_10098.pt`
    deterministic eval:
    `physical_turn_motion_score = 0.3541501714724594`,
    `turn_motion_score = 0.519261335162446`,
    `clean_hold_rate = 0.984375`,
    `time_out_rate = 0.984375`,
    `bad_orientation_rate = 0.015625`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_yaw_ratio_symmetric = 0.11556387692689896`.
    So this first reward-only command-conditioned branch did not beat retained `M6l`. It slightly regressed the mixed-sign turn score, slightly reduced symmetric yaw alignment, and reintroduced a small orientation failure rate. The conclusion is specific: a fixed both-hard scaffold can support a clean mixed-sign turn baseline, but the first command-conditioned single-side reward shaping pass was not a useful lever. The retained mixed-turn rung stays `M6l model_10098.pt`.

146. I then tested whether the mixed-sign both-hard baseline simply needed more continuation rather than a new mechanism. A longer bounded model-only continuation from retained `M3f model_10049.pt` was launched on `M6l`:
    `unitree_g1_29dof_wheelchair_scratch_m6l_freeyaw_heavydamped_mixedturn_observed_both_hard_relaxedhandle/2026-05-26_13-21-47_mixedturn_bothhard_from_m3f10049_modelonly_200it`.
    I stopped it after the `50/100/150` checkpoint ladder was already written, because that was enough to answer the selection question. The deterministic eval results were:
    - `model_10050.pt`:
      `physical_turn_motion_score = 0.3518808914450896`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.1114727109670639`,
      `clean_hold_rate = 1.0`,
      `invalid_contact_rate = 0.0`
    - `model_10100.pt`:
      `physical_turn_motion_score = 0.35305543622110797`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.09481938183307648`,
      `clean_hold_rate = 0.984375`,
      `invalid_contact_rate = 0.0`
    - `model_10150.pt`:
      `physical_turn_motion_score = 0.3413121377635986`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.08664043247699738`,
      `clean_hold_rate = 0.984375`,
      `invalid_contact_rate = 0.015625`
    None of those beat the earlier retained short-run checkpoint `M6l model_10098.pt`, which stayed at
    `physical_turn_motion_score = 0.3666192910436557`
    and
    `wheelchair_command_aligned_yaw_ratio_symmetric = 0.12723684310913086`
    with `clean_hold_rate = 1.0` and `invalid_contact_rate = 0.0`.
    So the longer same-task continuation is not the right lever. `M6l` does improve slightly from zero-shot, but then drifts backward with more training. The retained mixed-sign turn rung remains the short `50`-iteration `M6l model_10098.pt` result, and the next useful branch should change task structure again rather than just train `M6l` longer.

147. The next task-structure branch was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6n-FreeYawHeavyDampedMixedTurn-RapidResample-Observed-BothHard-RelaxedHandle`.
    `M6n` kept the exact same both-hard heavy-damped scaffold as retained `M6l`, but changed the command schedule so the yaw command resampled every `2.0 s` inside the episode instead of staying fixed for the whole rollout. The point was to force sign-switch experience instead of letting the policy treat left and right turns as separate static episodes.
    I launched a matched `50`-iteration model-only continuation from retained `M6l model_10098.pt`:
    `unitree_g1_29dof_wheelchair_scratch_m6n_freeyaw_heavydamped_mixedturn_rapidresample_observed_both_hard_relaxedhandle/2026-05-26_13-38-42_rapidresample_from_m6l10098_modelonly_50it`
    and evaluated the resulting `model_10100.pt` back on the standard downstream `M6l` mixed-turn task. The downstream deterministic eval came back at:
    `physical_turn_motion_score = 0.3448920971138997`,
    `turn_motion_score = 0.5191973022185267`,
    `clean_hold_rate = 0.984375`,
    `time_out_rate = 0.984375`,
    `base_height_rate = 0.015625`,
    `invalid_contact_rate = 0.0`,
    and `wheelchair_command_aligned_yaw_ratio_symmetric = 0.11264941841363907`.
    So the rapid intra-episode sign-switch curriculum also failed to beat retained `M6l model_10098.pt`. It stayed physically clean, but it regressed both the mixed-turn physical score and the symmetric yaw-alignment score on the standard task. The conclusion is again narrow: the unified-turn blocker is not just lack of command switching within the episode. The retained mixed-turn rung still stays `M6l model_10098.pt`, and the next branch needs a different control or topology mechanism rather than more schedule tweaks on the same both-hard scaffold.

148. I then tightened the evaluator instead of guessing from the aggregate mixed-turn score. `eval_wheelchair_m0.py` now emits `turn_metrics_by_command_sign`, splitting the turn metrics into `left_turn_command` and `right_turn_command` subsets. I reran the retained mixed-turn baseline `M6l model_10098.pt` on the standard `M6l` task with the full episode horizon (`64 envs`, `500 steps`). The split result is decisive:
    - `left_turn_command`:
      `physical_turn_motion_score = 0.6922063695049961`,
      `turn_motion_score = 1.3198288734070955`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.8864503502845764`,
      `wheelchair_yaw_velocity_mean = 0.459000825881958`,
      `clean_hold_rate = 1.0`,
      `invalid_contact_rate = 0.0`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.07233190653455447`,
      `turn_motion_score = -0.19419106543064119`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = -0.585963785648346`,
      `wheelchair_yaw_velocity_mean = 0.3227216303348541`,
      `clean_hold_rate = 1.0`,
      `invalid_contact_rate = 0.0`
    So `M6l` is not a uniformly weak mixed-sign controller. It is effectively the retained left-turn specialist plus a still-broken right-turn branch on the same both-hard scaffold. The aggregate retained score
    `physical_turn_motion_score = 0.3666192910436557`
    is mostly just the average of one good sign and one wrong sign. That narrows the next experiment substantially: the next unified-turn branch should target right-turn sign correction directly, not perturb the whole mixed-turn task globally.

149. I tested that exact mirrored branch next instead of guessing: a mixed-sign turn task on the retained right-turn scaffold,
    `Unitree-G1-29dof-Wheelchair-Scratch-M6o-FreeYawHeavyDampedMixedTurn-Observed-RightHardLeftSoft-RelaxedHandle`.
    `M6o` kept the same heavy-damped turn dynamics and symmetric yaw-command range as `M6l`, but replaced the both-hard grip with the retained right-hard/left-soft turn topology from `M6c`. I ran a matched `50`-iteration model-only continuation from retained `M3f model_10049.pt`:
    `logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6o_freeyaw_heavydamped_mixedturn_observed_right_hard_left_soft_relaxedhandle/2026-05-26_13-56-11_mixedturn_righthardleftsoft_from_m3f10049_modelonly_50it`
    and evaluated the saved checkpoints with the split-turn metrics.
    The later checkpoint `model_10098.pt` was slightly better than `model_10050.pt`, but the result simply mirrored `M6l` instead of solving it:
    - aggregate:
      `physical_turn_motion_score = 0.3394886074186614`,
      `turn_motion_score = 0.4556515691103414`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = -0.02971457690000534`,
      `clean_hold_rate = 1.0`,
      `invalid_contact_rate = 0.0`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.03331444319650689`,
      `turn_motion_score = -0.3082868215395137`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = -0.7320753931999207`,
      `wheelchair_yaw_velocity_mean = -0.2430439293384552`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.7535063807269622`,
      `turn_motion_score = 1.3776461312081665`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.8179622292518616`,
      `wheelchair_yaw_velocity_mean = -0.3072325587272644`
    So `M6o` is the exact mirror image of retained `M6l`: strong right turn, wrong-sign left turn, fully stable, and fully contact-clean. It does not beat retained `M6l` as a unified mixed-turn controller, so `M6o` is discarded as a fixed-scaffold mixed branch and removed from code. The lesson is now concrete: neither fixed both-hard nor fixed right-hard/left-soft can support symmetric mixed-sign turning. The next unified-turn branch needs command-conditioned topology or a similar sign-aware mechanism, not another fixed attachment scaffold.

150. I then moved to the first command-conditioned mixed-turn scaffold that stays inside the mechanically proven task family instead of switching attachment topology at reset time:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6p-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedSoft-RelaxedHandle`.
    `M6p` replaces the fixed both-hard grip with two bounded soft hand-handle attachments whose dominant side follows the commanded yaw sign at runtime:
    - left-turn command: left side dominant, right side assist
    - right-turn command: right side dominant, left side assist
    - near-zero yaw: both sides neutral
    The zero-shot transfer from retained `M6l model_10098.pt` was immediately useful because it changed the sign behavior without reopening posture failure:
    - aggregate:
      `physical_turn_motion_score = 0.09044851200650537`,
      `turn_motion_score = 0.6346616450930015`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.11344851553440094`,
      `clean_hold_rate = 0.921875`,
      `time_out_rate = 1.0`,
      `bad_orientation_rate = 0.0`,
      `invalid_contact_rate = 0.078125`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.04624660266167488`,
      `turn_motion_score = 0.5735906860558317`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.0562153197824955`,
      `wheelchair_yaw_velocity_mean = 0.010243130847811699`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.14185192968349883`,
      `turn_motion_score = 0.708367930725217`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.18252304196357727`,
      `wheelchair_yaw_velocity_mean = -0.02506144717335701`
    That is the first mixed-sign branch here where both yaw-command signs stayed physically valid and produced the correct yaw sign on one shared runtime mechanism. But authority was still extremely weak, and invalid contact was still real.

151. I then ran a bounded `50`-iteration model-only continuation on `M6p` from retained `M6l model_10098.pt`:
    `logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6p_freeyaw_heavydamped_mixedturn_observed_command_conditioned_soft_relaxedhandle/2026-05-26_14-13-42_mixedturn_commandsoft_from_m6l10098_modelonly_50it`
    and evaluated both saved checkpoints. The later checkpoint `model_10147.pt` was the better one:
    - aggregate:
      `physical_turn_motion_score = 0.11287512941327117`,
      `turn_motion_score = 0.680165586457588`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.1318761706352234`,
      `clean_hold_rate = 0.984375`,
      `time_out_rate = 1.0`,
      `bad_orientation_rate = 0.0`,
      `invalid_contact_rate = 0.015625`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.056229882400059016`,
      `turn_motion_score = 0.6117976468289271`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.06599722802639008`,
      `wheelchair_yaw_velocity_mean = 0.01132544968277216`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.17932415988238903`,
      `turn_motion_score = 0.7626786550041288`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.21138525009155273`,
      `wheelchair_yaw_velocity_mean = -0.02904720976948738`
    Training therefore improved `M6p` in the right direction: higher aggregate physical turn score, better symmetric yaw alignment, and much lower invalid contact than zero-shot. But it still does not beat retained `M6l model_10098.pt` as a mixed-turn milestone, and its left-turn authority is still far below the retained left-turn branch. So `M6p` is worth keeping as the first command-conditioned sign-correct unified-turn foothold, but not as the retained mixed-turn controller. The next useful branch should strengthen command-conditioned authority rather than go back to another fixed scaffold.

152. I then tested exactly that stronger command-conditioned branch:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6q-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RelaxedHandle`.
    `M6q` keeps the same `M6p` runtime mechanism, but makes the commanded dominant side much stiffer and the assist side much weaker. Zero-shot from retained `M6p model_10147.pt` immediately showed the right tradeoff: yaw authority increased substantially, but the right-turn side reopened contact and posture failures:
    - aggregate zero-shot:
      `physical_turn_motion_score = 0.180478867037474`,
      `turn_motion_score = 0.7685924386605621`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.4062581956386566`,
      `clean_hold_rate = 0.765625`,
      `bad_orientation_rate = 0.140625`,
      `invalid_contact_rate = 0.234375`
    A short bounded continuation from `M6p model_10147.pt` improved the branch materially. The better checkpoint was `model_10150.pt`:
    - aggregate:
      `physical_turn_motion_score = 0.21757397106793339`,
      `turn_motion_score = 0.7585547987371685`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.3074813783168793`,
      `clean_hold_rate = 0.828125`,
      `time_out_rate = 0.953125`,
      `bad_orientation_rate = 0.015625`,
      `invalid_contact_rate = 0.171875`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.18983607529271296`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.24410340189933777`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.21480752041858034`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.38397204875946045`,
      `clean_hold_rate = 0.6896551847457886`,
      `invalid_contact_rate = 0.3103448152542114`
    This is the first branch here where both left and right turn commands have clearly positive physical-turn scores on one shared mechanism. But it is not retainable as a milestone because the right-turn half is still too dirty.

153. I then checked whether that remaining `M6q` failure could be cleaned up with reward-side pressure instead of another mechanism change:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6r-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-ContactClean-RelaxedHandle`.
    `M6r` keeps the `M6q` mechanism intact and only increases `wheelchair_invalid_contact` and `wheelchair_robot_standoff` penalties. That did not solve the real problem. The better checkpoint was `model_10150.pt`:
    - aggregate:
      `physical_turn_motion_score = 0.2020381042806937`,
      `turn_motion_score = 0.7551943441852926`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.3301670253276825`,
      `clean_hold_rate = 0.8125`,
      `time_out_rate = 0.875`,
      `bad_orientation_rate = 0.078125`,
      `invalid_contact_rate = 0.171875`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.19608352782752075`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.2514706254005432`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.15149955009310193`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.4251454472541809`,
      `clean_hold_rate = 0.6551724076271057`,
      `bad_orientation_rate = 0.17241379618644714`,
      `invalid_contact_rate = 0.3103448152542114`
    So `M6r` confirmed the failure surface rather than fixing it: stronger command-conditioned soft dominance can create usable bidirectional yaw authority, but simply turning up invalid-contact and standoff penalties does not buy back the right-turn cleanliness. The blocker is now narrower than before: the next mixed-turn branch should keep the sign-conditioned authority idea from `M6q`, but solve the right-turn contact geometry with a different physical scaffold rather than more penalty weight on the same setup.

154. I also tested a narrower reward-only follow-up on top of retained `M6p model_10147.pt`:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6s-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedSoft-DominantGeometry-RelaxedHandle`.
    `M6s` kept the original `M6p` soft command-conditioned turn scaffold, but replaced the shared hand-handle geometry shaping with commanded dominant-side-only geometry rewards. The idea was to preserve the clean `M6p` runtime mechanism while giving the commanded turn side a clearer spatial target without jumping all the way to the stronger `M6q` authority settings.
    The bounded `50`-iteration model-only continuation run was:
    `logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6s_freeyaw_heavydamped_mixedturn_observed_command_conditioned_soft_dominant_geometry_relaxedhandle/2026-05-26_14-58-24_mixedturn_commandsoft_dominantgeom_from_m6p10147_modelonly_50it`
    and produced one saved checkpoint, `model_10150.pt`.
    Compared against retained `M6p model_10147.pt`, `M6s model_10150.pt` regressed slightly on the aggregate mixed-turn gate:
    - retained `M6p model_10147.pt`:
      `physical_turn_motion_score = 0.11287512941327117`,
      `turn_motion_score = 0.680165586457588`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.1318761706352234`,
      `clean_hold_rate = 0.984375`,
      `invalid_contact_rate = 0.015625`
    - `M6s model_10150.pt`:
      `physical_turn_motion_score = 0.11120020173864448`,
      `turn_motion_score = 0.6665303901769223`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.12793606519699097`,
      `clean_hold_rate = 0.96875`,
      `invalid_contact_rate = 0.03125`
    The sign split makes the failure mode more specific:
    - `left_turn_command` improved a little on authority
      (`physical_turn_motion_score = 0.07252853426676785` vs `0.056229882400059016`)
      but reopened contact
      (`invalid_contact_rate = 0.02857142873108387` vs `0.0`)
    - `right_turn_command` got worse on authority
      (`physical_turn_motion_score = 0.158437060450738` vs `0.17932415988238903`)
      while staying just as dirty as retained `M6p`
      (`invalid_contact_rate = 0.03448275849223137`)
    Aggregate invalid contact also shifted the failure surface the wrong way:
    - retained `M6p`: `wheelchair_base_robot_contact = 0.015625`, `wheelchair_right_handle_invalid_contact = 0.015625`
    - `M6s`: `wheelchair_base_robot_contact = 0.03125`, `wheelchair_left_handle_invalid_contact = 0.015625`
    So `M6s` did not solve the mixed-turn bottleneck. Dominant-side-only geometry shaping is not enough on top of the original soft scaffold. I discarded `M6s` from code. The next mixed-turn branch should continue from the `M6q` lesson instead: preserve sign-conditioned authority, but change the right-turn physical scaffold or contact geometry rather than just reweighting geometry rewards inside the weaker `M6p` mechanism.

155. I then restored the stronger `M6q` mixed-turn scaffold into the live codebase because it had only existed in saved run artifacts, not in the current source tree:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6q-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RelaxedHandle`.
    The first reconstruction pass exposed a real runtime bug in the helper path: `ScratchM6q...` was calling `_apply_command_conditioned_turn_soft_handle_scaffold(...)` with explicit dominant/assist gain overrides, but the current helper implementation only accepted the original fixed signature. That meant the task could compile but not instantiate. After restoring the missing helper parameters, I reran the deterministic zero-shot eval from retained `M6p model_10147.pt` and recovered the historical `M6q` zero-shot branch exactly:
    - aggregate zero-shot:
      `physical_turn_motion_score = 0.180478867037474`,
      `turn_motion_score = 0.7685924386605621`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.4062581956386566`,
      `clean_hold_rate = 0.765625`,
      `bad_orientation_rate = 0.140625`,
      `invalid_contact_rate = 0.234375`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.1849697791430749`,
      `invalid_contact_rate = 0.02857142873108387`
    - `right_turn_command`:
      `physical_turn_motion_score = -0.008921457944580906`,
      `invalid_contact_rate = 0.48275861144065857`
    So the code restoration is now mechanically verified: the live `M6q` implementation reproduces the historical zero-shot behavior and remains the correct stronger shared-turn scaffold to branch from.

156. The next bounded follow-up on top of that restored scaffold was:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6u-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-DominantGeometry-RelaxedHandle`.
    `M6u` kept the stronger `M6q` command-conditioned soft dominance and reintroduced dominant-side-only hand-handle geometry shaping, with the specific goal of cleaning up the right-turn contact leak without giving back the both-sign authority. As expected, zero-shot from `M6p model_10147.pt` was identical to zero-shot `M6q` because only the reward changed. The real test was a matched bounded `50`-iteration model-only continuation:
    `logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6u_freeyaw_heavydamped_mixedturn_observed_command_conditioned_strong_soft_dominant_geometry_relaxedhandle/2026-05-26_16-07-39_mixedturn_commandstrongsoft_dominantgeom_from_m6p10147_modelonly_50it`
    which produced two saved checkpoints:
    - `model_10150.pt`:
      `physical_turn_motion_score = 0.203279605285046`,
      `turn_motion_score = 0.7342491181567311`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.32976973056793213`,
      `clean_hold_rate = 0.796875`,
      `bad_orientation_rate = 0.03125`,
      `invalid_contact_rate = 0.203125`
      with `right_turn_command physical_turn_motion_score = 0.14647181343215585`
      and `right_turn_command invalid_contact_rate = 0.41379308700561523`
    - `model_10196.pt`:
      `physical_turn_motion_score = 0.19345759608092344`,
      `turn_motion_score = 0.7403149347752332`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.31351807713508606`,
      `clean_hold_rate = 0.828125`,
      `bad_orientation_rate = 0.09375`,
      `invalid_contact_rate = 0.140625`
      with `right_turn_command physical_turn_motion_score = 0.13699392383603276`
      and `right_turn_command invalid_contact_rate = 0.27586206793785095`
    This is a real improvement over zero-shot `M6q` because the right-turn half becomes physically positive instead of wrong-sign. But it still does not beat the historical bounded `M6q` continuation on the actual tradeoff we care about. Historical `M6q model_10150.pt` stayed stronger on authority (`physical_turn_motion_score = 0.21757397106793339`, right-turn `0.21480752041858034`) while landing at a comparable aggregate cleanliness point (`clean_hold_rate = 0.828125`, `invalid_contact_rate = 0.171875`). `M6u` buys back some right-turn contact, but it does so by giving back too much turn authority and reintroducing more bad orientation on the later checkpoint. So `M6u` is discarded from code. The retained lesson is narrower: preserving `M6q` sign-conditioned authority is correct, but dominant-side-only geometry shaping is still not the mechanism that resolves the right-turn scaffold cleanly.

157. I then fixed the mixed-turn evaluator itself so the diagnosis path could be trusted. The per-handle/per-sensor breakdown mode in
    `scripts/rsl_rl/eval_wheelchair_m0.py`
    had been preallocating filter tensors from the static config lists, but the live relaxed-handle sensors were returning a different filter count at runtime. That caused the diagnosis run to crash with a `31 != 30` tensor-width mismatch instead of telling us what was actually colliding. I changed that evaluator path to size breakdown buffers from the live sensor shapes and to label them from the runtime filter expressions when available. After that fix, the historical bounded `M6q model_10150.pt` breakdown became usable and showed the real right-turn leak:
    - `wheelchair_right_handle_invalid_contact` was dominated by
      `right_wrist_pitch_link = 27904.484375`
    - the next contributors were much smaller:
      `right_wrist_roll_link = 237.5985107421875`,
      `right_elbow_link = 124.99657440185547`,
      `pelvis = 77.53118896484375`
    - `wheelchair_base_robot_contact` was also mostly hand/wrist spill:
      `right_rubber_hand = 380.507568359375`,
      `right_wrist_pitch_link = 353.1176452636719`,
      `right_wrist_yaw_link = 320.6406555175781`
    Based on that, I ran one more bounded probe:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6v-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-DominantWrist-RelaxedHandle`.
    `M6v` kept the strong-soft `M6q` scaffold, started from the historical bounded
    `M6q model_10150.pt`,
    and added a dominant-side wrist pitch/roll deviation term with the explicit goal of reducing the `right_wrist_pitch_link` handle intrusion without giving away two-sign turn authority.
    The bounded continuation run was:
    `unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6v_freeyaw_heavydamped_mixedturn_observed_command_conditioned_strong_soft_dominant_wrist_relaxedhandle/2026-05-26_16-34-03_mixedturn_commandstrongsoft_dominantwrist_from_m6q10150_modelonly_50it`
    and produced `model_10199.pt` as the only new saved checkpoint.
    Deterministic eval on `M6v model_10199.pt` came back at:
    - aggregate:
      `physical_turn_motion_score = 0.21214263804737943`,
      `turn_motion_score = 0.7641600643284618`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.32313433289527893`,
      `clean_hold_rate = 0.828125`,
      `invalid_contact_rate = 0.171875`,
      `bad_orientation_rate = 0.046875`,
      `base_height_rate = 0.078125`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17840703906227545`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.24167948961257935`,
      `clean_hold_rate = 0.9142857193946838`,
      `invalid_contact_rate = 0.08571428805589676`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.21401638972764173`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.42144185304641724`,
      `clean_hold_rate = 0.7241379022598267`,
      `invalid_contact_rate = 0.27586206793785095`
    Relative to historical bounded `M6q model_10150.pt`, this did exactly what the targeted wrist penalty was supposed to do on the weak side:
    - right-turn invalid contact improved:
      `0.3103448152542114 -> 0.27586206793785095`
    - right-turn clean hold improved:
      `0.6896551847457886 -> 0.7241379022598267`
    - and the dominant leak body was cut almost in half:
      `wheelchair_right_handle_invalid_contact -> right_wrist_pitch_link`
      `27904.484375 -> 13843.240234375`
    But it was still not a win overall. Aggregate `physical_turn_motion_score` regressed slightly
    (`0.21757397106793339 -> 0.21214263804737943`),
    aggregate `invalid_contact_rate` stayed exactly flat at `0.171875`,
    and the contact mostly shifted into adjacent geometry rather than disappearing:
    `wheelchair_base_robot_contact -> right_rubber_hand`
    rose to `735.692138671875`,
    while `right_wrist_roll_link` also rose to `469.6550598144531`.
    So `M6v` confirmed a useful physical fact: the dominant-side wrist term can reduce the specific `right_wrist_pitch_link` handle strike, but by itself it does not solve the mixed-turn scaffold. It just trades that leak for neighboring hand/base contact and degrades the left-turn half enough to lose overall. I discarded `M6v` from code. The retained outcome of this round is the evaluator fix plus a narrower diagnosis: the next `M6q`-family branch should change the dominant-side physical contact scaffold, not just add more reward pressure on the same soft geometry.

156. I then tested a more structural command-conditioned hybrid branch:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6w-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedHybrid-RelaxedHandle`.
    The intent was to use a hard dominant-side hand-handle joint on the commanded turn side and keep a soft assist on the other side, switching that topology with the yaw-command sign. The zero-shot transfer from retained `M6q model_10150.pt` immediately showed that this was not a real control improvement even before training:
    `physical_turn_motion_score = 0.0035975821035016385`,
    `turn_motion_score = 0.4383784196572379`,
    `wheelchair_command_aligned_yaw_ratio_symmetric = -0.1019144207239151`,
    `clean_hold_rate = 1.0`,
    and `invalid_contact_rate = 0.0`.
    So `M6w` was physically clean but essentially non-turning and wrong-sign.
    I still ran a bounded continuation attempt from retained `M6q model_10150.pt`, but that branch exposed a more serious issue: reset-time USD joint mutation on this scaffold triggered Isaac/PhysX hierarchy/xformstack errors during training instead of producing a valid checkpoint ladder. I stopped that run and discarded `M6w` from code. The lesson is specific: command-conditioned reset-time hard/soft joint rewriting is not currently a mechanically valid mixed-turn scaffold in this environment stack.

157. The next branch kept the proven runtime mechanism from `M6q` and changed only the right-turn dominant-side strength:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6x-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedAsymmetricSoft-RelaxedHandle`.
    `M6x` kept the left-turn dominant side at the retained `M6q` strong-soft values, but softened the right-turn dominant side to reduce the specific right wrist/base intrusion seen in the `M6q` breakdown.
    Zero-shot transfer from retained `M6q model_10150.pt` came back at:
    - aggregate:
      `physical_turn_motion_score = 0.1523773732366186`,
      `turn_motion_score = 0.6750587449409067`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.1960894614458084`,
      `clean_hold_rate = 0.875`,
      `invalid_contact_rate = 0.125`,
      `bad_orientation_rate = 0.0`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.1895894598691404`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.24422724545001984`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.1128378109060148`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.13799212872982025`,
      `clean_hold_rate = 0.7931034564971924`,
      `invalid_contact_rate = 0.20689654350280762`
    That was directionally plausible: it was cleaner than retained `M6q`, especially on the right-turn side, but it gave back a lot of turn authority.
    I then ran a matched bounded `50`-iteration model-only continuation from retained `M6q model_10150.pt`:
    `unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6x_freeyaw_heavydamped_mixedturn_observed_command_conditioned_asymmetric_soft_relaxedhandle/2026-05-26_17-10-40_mixedturn_asymmetricsoft_from_m6q10150_modelonly_50it`
    The run produced one new checkpoint, `model_10199.pt`, and it regressed relative to its own zero-shot start:
    - aggregate:
      `physical_turn_motion_score = 0.13453308225478522`,
      `turn_motion_score = 0.6417798833455891`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.18083100020885468`,
      `clean_hold_rate = 0.84375`,
      `invalid_contact_rate = 0.15625`,
      `bad_orientation_rate = 0.0`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17649054176165396`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.22434625029563904`,
      `clean_hold_rate = 0.9142857193946838`,
      `invalid_contact_rate = 0.08571428805589676`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.09141481387408519`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.1283126175403595`,
      `clean_hold_rate = 0.7586206793785095`,
      `invalid_contact_rate = 0.24137930572032928`
    Compared with retained historical bounded `M6q model_10150.pt`, `M6x` is slightly cleaner in aggregate (`0.15625` vs `0.171875` invalid-contact rate) but materially weaker on the real turn objective (`0.1345` vs `0.2176` physical turn score), and it loses most of that gap on the right-turn half. So `M6x` is not retained and was removed from code. The useful conclusion is narrower than before: softening the weak-side dominant attachment can buy some cleanliness, but on this scaffold it buys it by giving away too much two-sign turn authority.

158. I then tested the opposite-side support hypothesis directly:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6y-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-LeftAssist-RelaxedHandle`.
    `M6y` kept the retained `M6q` dominant-side strength intact, but increased the left-side assist used during right-turn episodes. The intent was to preserve right-turn authority while letting the off-side hand help stabilize the torso and keep the dominant right wrist out of the chair.
    That branch failed immediately on zero-shot transfer from retained `M6q model_10150.pt`, so I did not run a continuation. Deterministic zero-shot eval came back at:
    - aggregate:
      `physical_turn_motion_score = 0.17580272792821708`,
      `turn_motion_score = 0.7473227627575397`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.3668012022972107`,
      `clean_hold_rate = 0.78125`,
      `invalid_contact_rate = 0.21875`,
      `bad_orientation_rate = 0.140625`,
      `base_height_rate = 0.046875`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17390093406513527`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.22297799587249756`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.05574707816595314`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.5403808951377869`,
      `clean_hold_rate = 0.5862069129943848`,
      `invalid_contact_rate = 0.41379308700561523`,
      `bad_orientation_rate = 0.3103448152542114`
    The right-turn sign stayed positive, but the physical base score collapsed because the branch immediately reintroduced large invalid contact, bad orientation, and base-height failures on the weak side. So the increased opposite-side assist is not the fix either. I discarded `M6y` from code without spending a train budget on it.

159. The next geometry-only probe changed the dominant right palm grip point itself:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6z-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightGripOutboardRaised-RelaxedHandle`.
    `M6z` kept the retained `M6q` strong-soft gains exactly as they were, but moved the right palm grip point farther forward, outward, and upward on the `right_rubber_hand` body during mixed-turn control. The goal was to reduce the right-turn `right_wrist_pitch_link` handle strike without weakening right-turn authority the way `M6x` did.
    This branch did not produce a usable zero-shot eval result. The rollout never wrote `/tmp/m6z_zeroshot_eval.json`, and the active Kit log
    `kit_20260526_172942.log`
    reported repeated
    `Hang detected`
    events during the same run instead of completing the deterministic evaluation. I stopped the probe and removed `M6z` from code.
    So the current lesson is that this larger right-palm grip relocation is not just unproven; in the present stack it is mechanically unstable enough to hang the zero-shot rollout before metrics are emitted.

160. I then tested a smaller geometry-only right-side scaffold change:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6aa-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightHandleForwardUp-RelaxedHandle`.
    `M6aa` kept the retained `M6q` strong-soft gains unchanged and did not move the robot hand grip point. Instead it shifted only the right handle target point slightly forward and upward in `right_handle_frame` local coordinates:
    `right_wheelchair_body_local_positions = [[0.008, 0.0, 0.012]]`.
    The intent was to keep the successful `M6q` authority mechanism intact while making the dominant right-turn alignment a little less wrist-pitch-heavy than the original contact geometry.
    This branch also failed before producing a usable zero-shot eval. The smoke run never wrote `/tmp/m6aa_smoke_eval.json`, the Isaac log
    `isaaclab_2026-05-26_17-40-20.log`
    stopped at environment/reward-manager initialization, and the live Kit log
    `kit_20260526_174007.log`
    showed:
    `SimulationApp.close: Closing application`
    followed roughly two minutes later by:
    `Hang detected`
    and a declined crash dialog. So `M6aa` reproduced the same failure class as `M6z`: a geometry-only right-side handle-target change that is mechanically unstable enough to hang shutdown before deterministic eval metrics are emitted.
    I removed `M6aa` from code. The mixed-turn lesson is tighter now: large right-palm relocation and smaller right-handle-target relocation both fail mechanically in the current stack, so the next `M6q`-family branch should stay away from more right-side geometry rewiring and instead change a different part of the right-turn physical scaffold.

161. I then tested whether the dominant weak-side leak was really just a handle-contact classification issue:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6ab-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-PitchRelaxedHandle`.
    `M6ab` kept the retained `M6q` strong-soft scaffold unchanged, but temporarily relaxed handle validity to allow same-side `wrist_pitch` contact at both handles in addition to the already-allowed `rubber_hand` and `wrist_yaw` links. The idea was to see whether the right-turn failure was mostly a bookkeeping problem around `right_wrist_pitch_link` touching the handle rather than a real base/torso contact problem.
    Deterministic zero-shot eval from retained `M6q model_10150.pt` came back at:
    - aggregate:
      `physical_turn_motion_score = 0.19427912372222916`,
      `turn_motion_score = 0.7051787175238131`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.38512715697288513`,
      `clean_hold_rate = 0.8125`,
      `invalid_contact_rate = 0.1875`,
      `bad_orientation_rate = 0.046875`,
      `base_height_rate = 0.0625`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17755345293315306`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.24824529886245728`,
      `clean_hold_rate = 0.9142857193946838`,
      `invalid_contact_rate = 0.08571428805589676`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.16123353407766905`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.5503294467926025`,
      `clean_hold_rate = 0.6551724076271057`,
      `invalid_contact_rate = 0.3448275923728943`,
      `bad_orientation_rate = 0.13793103396892548`
    The useful detail is in the sensor split. `M6ab` did suppress the specific handle invalid-contact terms hard: `wheelchair_right_handle_invalid_contact` dropped to `4182.55810546875` mean force and `0.125` rate, while `wheelchair_left_handle_invalid_contact` dropped to `595.6119384765625` mean force and `0.09375` rate. But the aggregate branch still lost to retained `M6q` because the contact just migrated into `wheelchair_base_robot_contact`, which stayed large at `6263.53466796875` mean force with `0.1875` rate. So `M6ab` proved the weak side was not primarily misclassified handle contact. It was a real physical collapse into the chair base once the wrist-pitch link stopped being counted against the handles. I discarded `M6ab` from code without spending a training budget on it.

162. I then tested a narrow reward-side cleanup targeted only at the surviving weak-side sensors:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6ac-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightTurnContactClean-RelaxedHandle`.
    `M6ac` kept the retained `M6q` strong-soft attachment scaffold intact and added one extra reward term only during right-turn-command episodes: a filtered invalid-contact penalty on `wheelchair_base_robot_contact` and `wheelchair_right_handle_invalid_contact`.
    I ran a matched `50`-iteration model-only continuation from retained `M6q model_10150.pt`:
    `unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6ac_freeyaw_heavydamped_mixedturn_observed_command_conditioned_strong_soft_rightturn_contactclean_relaxedhandle/2026-05-26_17-55-28_mixedturn_rightturncontact_from_m6q10150_modelonly_50it`
    The run produced `model_10150.pt` and `model_10199.pt`; deterministic eval of the later checkpoint came back at:
    - aggregate:
      `physical_turn_motion_score = 0.2022933866306722`,
      `turn_motion_score = 0.7843994962051511`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.38512715697288513`,
      `clean_hold_rate = 0.796875`,
      `invalid_contact_rate = 0.203125`,
      `bad_orientation_rate = 0.078125`,
      `base_height_rate = 0.078125`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17755345293315306`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.24824529886245728`,
      `clean_hold_rate = 0.9142857193946838`,
      `invalid_contact_rate = 0.08571428805589676`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.16123353407766905`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.5503294467926025`,
      `clean_hold_rate = 0.6551724076271057`,
      `invalid_contact_rate = 0.3448275923728943`,
      `bad_orientation_rate = 0.13793103396892548`
    Relative to retained historical bounded `M6q model_10150.pt`, `M6ac` did not solve the actual tradeoff. Aggregate `physical_turn_motion_score` regressed (`0.2023` vs `0.2176`), aggregate `invalid_contact_rate` got worse (`0.203125` vs `0.171875`), and the right-turn half still stayed materially worse on both cleanliness and physical base score. In other words, extra right-turn-only penalty pressure on the already-identified weak-side sensors was not enough to clean the scaffold without also suppressing useful turn behavior. I discarded `M6ac` from code and kept retained `M6q model_10150.pt` as the active mixed-turn checkpoint.

163. I then tested a narrower physical-scaffold change inside the same `M6q` mechanism:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6ad-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightForceCapped-RelaxedHandle`.
    `M6ad` kept the retained `M6q` command-conditioned strong-soft scaffold and changed only one thing: during right-turn-command episodes, the dominant right-hand soft attachment kept the same stiffness/damping but lowered `max_force` from `1000.0` to `700.0`. The intent was to preserve the useful two-sign authority from `M6q` while reducing the specific right-turn chair intrusion without touching rewards or geometry.
    Deterministic zero-shot transfer from retained `M6q model_10150.pt` came back at:
    - aggregate:
      `physical_turn_motion_score = 0.19002076231086962`,
      `turn_motion_score = 0.6953225670315612`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.2520219683647156`,
      `clean_hold_rate = 0.875`,
      `invalid_contact_rate = 0.125`,
      `base_height_rate = 0.015625`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.186151182969205`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.1883712823501326`,
      `clean_hold_rate = 0.7931034564971924`,
      `invalid_contact_rate = 0.20689654350280762`
    That zero-shot probe was cleaner than retained `M6q`, especially on the right-turn half, so I spent one matched bounded continuation budget from the same retained source:
    `unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_wheelchair_scratch_m6ad_freeyaw_heavydamped_mixedturn_observed_command_conditioned_strong_soft_rightforcecapped_relaxedhandle/2026-05-26_18-21-17_mixedturn_rightforcecapped_from_m6q10150_modelonly_50it`
    The saved checkpoints were `model_10150.pt` and `model_10199.pt`. Deterministic eval selected `model_10150.pt` as the better one:
    - aggregate:
      `physical_turn_motion_score = 0.18106631314477534`,
      `turn_motion_score = 0.7072188844904302`,
      `wheelchair_command_aligned_yaw_ratio_symmetric = 0.27011001110076904`,
      `clean_hold_rate = 0.8125`,
      `invalid_contact_rate = 0.1875`,
      `base_height_rate = 0.03125`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.19664195016651886`,
      `clean_hold_rate = 0.8857142925262451`,
      `invalid_contact_rate = 0.11428571492433548`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.16070869259009296`,
      `clean_hold_rate = 0.7241379022598267`,
      `invalid_contact_rate = 0.27586206793785095`
    Relative to retained historical bounded `M6q model_10150.pt`, `M6ad` did not hold its zero-shot cleanliness advantage once trained. Aggregate `physical_turn_motion_score` regressed (`0.1811` vs `0.2176`), aggregate `clean_hold_rate` regressed (`0.8125` vs `0.828125`), and aggregate `invalid_contact_rate` was still slightly worse (`0.1875` vs `0.171875`). The right-turn half also stayed worse than retained `M6q` on both cleanliness and physical turn score. So lowering only the right-turn dominant-side force cap was not a useful lever. I discarded `M6ad` from code and kept retained `M6q model_10150.pt` as the active mixed-turn checkpoint.

164. I then checked whether the surviving weak-side contact was an underdamped dominant-side problem rather than a force-cap problem:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6ae-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightDamped-RelaxedHandle`.
    `M6ae` kept the retained `M6q` strong-soft scaffold and changed only one thing: during right-turn-command episodes, the dominant right-hand soft attachment kept the same stiffness and max force, but increased damping from `300.0` to `500.0`. The hypothesis was that the right-turn wrist/base spill might be an overshoot problem that could be reduced without giving back turn authority.
    That branch failed immediately on deterministic zero-shot transfer from retained `M6q model_10150.pt`, so I did not spend a train budget on it. The zero-shot eval came back at:
    - aggregate:
      `physical_turn_motion_score = 0.13812577034261808`,
      `clean_hold_rate = 0.6875`,
      `invalid_contact_rate = 0.3125`,
      `bad_orientation_rate = 0.125`,
      `time_out_rate = 0.828125`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17982053125865288`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = -0.07719530538038262`,
      `clean_hold_rate = 0.37931033968925476`,
      `invalid_contact_rate = 0.6206896305084229`,
      `bad_orientation_rate = 0.27586206793785095`
    So increasing only the right-turn dominant-side damping was not stabilizing. It actually collapsed the right-turn half back to wrong-sign behavior with much worse contact and posture. I discarded `M6ae` from code without training.

165. I then tested the opposite-side support hypothesis in the narrower direction:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6af-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightLowAssist-RelaxedHandle`.
    `M6af` kept the retained `M6q` dominant-side strength intact and changed only the right-turn off-side assist, reducing it from `250/20/80` to `100/10/30` on the left hand during right-turn-command episodes. The idea was that the off-side hand might be dragging the robot torso into the chair base while the dominant right side was trying to turn.
    Deterministic zero-shot transfer from retained `M6q model_10150.pt` came back at:
    - aggregate:
      `physical_turn_motion_score = 0.19930134763460872`,
      `clean_hold_rate = 0.8125`,
      `invalid_contact_rate = 0.1875`,
      `bad_orientation_rate = 0.0625`,
      `time_out_rate = 0.890625`
    - `left_turn_command`:
      `physical_turn_motion_score = 0.17590923426254665`,
      `clean_hold_rate = 0.9428571462631226`,
      `invalid_contact_rate = 0.05714285746216774`
    - `right_turn_command`:
      `physical_turn_motion_score = 0.15644299743092036`,
      `clean_hold_rate = 0.6551724076271057`,
      `invalid_contact_rate = 0.3448275923728943`,
      `bad_orientation_rate = 0.13793103396892548`
    This kept the right-turn sign physically positive, but it still did not beat retained `M6q model_10150.pt` on the real tradeoff. Aggregate `physical_turn_motion_score` regressed (`0.1993` vs `0.2176`), aggregate `clean_hold_rate` regressed (`0.8125` vs `0.828125`), and aggregate `invalid_contact_rate` worsened (`0.1875` vs `0.171875`). The right-turn half also stayed worse than retained `M6q` on both cleanliness and physical turn score. So weakening only the right-turn off-side assist was not a useful lever either. I discarded `M6af` from code without training.

166. I then tested a true physical-control-point branch instead of another spring retune:
    `Unitree-G1-29dof-Wheelchair-Scratch-M6ag-FreeYawHeavyDampedMixedTurn-Observed-CommandConditionedStrongSoft-RightWristYaw-RelaxedHandle`.
    `M6ag` kept the retained `M6q` strong-soft gains unchanged and changed only the weak-side control scaffold. I measured the existing retained palm grip point relative to `right_wrist_yaw_link` and used that local offset (`[0.09564, 0.00072, 0.00502]`) to drive the right handle from `right_wrist_yaw_link` instead of `right_rubber_hand`, while also allowing `right_wrist_yaw_link` as valid right-handle contact and updating the handle-state observations/rewards to use the mixed left-hand / right-wrist control pair.
    The task compiled and instantiated cleanly. In the Isaac logs, the reset/interval soft-attachment terms resolved `right_wrist_yaw_link` correctly, and the mixed `wheelchair_handle_state` observation resolved body names `['left_rubber_hand', 'right_wrist_yaw_link']`. But the deterministic zero-shot eval from retained `M6q model_10150.pt` never produced metrics:
    - eval process stayed alive at full CPU
    - `/tmp/wheelchair_bridge_eval_5ruv86xi.txt` remained empty
    - Kit logged `SimulationApp.close: Closing application`
      without emitting the usual benchmark JSON
    So `M6ag` was not a scored failure like `M6ae` or `M6af`; it was a mechanical hang after the right-side control-point swap. I stopped the eval, removed `M6ag` from code, and kept retained `M6q model_10150.pt` as the active mixed-turn checkpoint. The useful lesson is narrow: swapping the weak side from `right_rubber_hand` to `right_wrist_yaw_link` is mechanically unstable in the current stack even when the local grip point is measured from the retained geometry.

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
