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

## Immediate Starting Point

The current recommended starting point for the loop is:

1. Re-establish a clean collidable standing task where torso-bracing against the chair is explicitly penalized and visible.
2. Confirm that the best standing checkpoint is physically clean in deterministic playback before attempting release.
3. Only then move into braked or heavily damped free-chair hold.

The current failure mode to avoid is clear: a policy that looks upright in scalar metrics but is actually using body-chair contact or chair interpenetration as support.

## Autoresearch Harness

The first `codex-autoresearch` loop should target `M0`, not the later motion phases.

Current loop task:

`Unitree-G1-29dof-Wheelchair-Scratch-M0-CollidableStand-DirectObs`

Mechanical verify command:

```bash
conda run --no-capture-output -n isaaclab python scripts/autoresearch/benchmark_wheelchair_m0.py --metric m0_score
```

That command lives in `unitree_rl_lab` and does two things: it runs a short bounded training continuation on the current M0 task, then it evaluates the resulting checkpoint with a deterministic standing rollout. The primary score is `m0_score`, but the evaluator also records `clean_hold_rate`, `invalid_contact_rate`, `bad_orientation_rate`, and `base_height_rate`. Phase advancement should not be decided from `m0_score` alone; it is only the dense optimization signal for the loop.

The default verifier intentionally omits the per-handle invalid-contact filter breakdown. That breakdown is still available as a diagnosis-only path in the evaluator, but it is not part of the unattended loop because it is materially heavier than the aggregate metric path.

For the unattended background loop, use the metrics-only JSON variant instead so the runtime can keep `m0_score` as the primary metric while also enforcing acceptance gates on `clean_hold_rate` and `invalid_contact_rate`:

```bash
conda run --no-capture-output -n isaaclab python scripts/autoresearch/benchmark_wheelchair_m0.py --metrics-json-only
```
