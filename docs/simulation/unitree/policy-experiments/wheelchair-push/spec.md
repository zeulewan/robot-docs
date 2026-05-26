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
46. The current read is therefore sharper now. `M1d` is the retained best transition checkpoint, `M1e` is a useful finer transfer rung but not yet a retained checkpoint stage, and the standard short continuation still destabilizes the lighter bridge tasks. The next likely lever is either an even shorter or otherwise gentler continuation schedule on `M1e`, or one more small dynamics step before the fully light-damped task, not a broad reward rewrite.

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
