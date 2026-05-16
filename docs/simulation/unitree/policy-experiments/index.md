# Policy Experiments

This folder tracks policy experiments separately from the base Unitree training guides. Each subfolder should describe one policy task, what it changed from the upstream task, how it was trained, and what behavior was observed.

## Experiments

| Experiment | Task IDs | Status |
|---|---|---|
| [G1 running policy family](g1-running/index.md) | Multiple | Overview of the walking-to-running-to-sprint lineage. |
| [G1 running](g1-running/running.md) | `Unitree-G1-29dof-Running` | Completed first running pass; checkpoint lineage ends at `model_2100.pt`. |
| [G1 fast running](g1-running/fast-running.md) | `Unitree-G1-29dof-Running-Fast` | Completed fast pass to `model_5099.pt`; used as the sprint warm-start. |
| [G1 sprint 10 m/s](g1-running/sprint-10ms.md) | `Unitree-G1-29dof-Sprint-10ms` | Paused at `model_20500.pt`; curriculum reached about `7.2 m/s`, gait needs tuning. |
| [G1 sprint gait cleanup](g1-running/sprint-10ms.md#gait-cleanup-variant) | `Unitree-G1-29dof-Sprint-10ms-Gait` | New stability-gated sprint variant for improving the running gait. |
| [G1 wheelchair push demo](wheelchair-push-demo.md) | `Unitree-G1-29dof-Velocity` | Visual demo using the walking policy with a kinematic wheelchair prop. |
| [G1 wheelchair handle-grip training](wheelchair-push-demo.md#handle-grip-training-task) | `Unitree-G1-29dof-Wheelchair-Push` | Warm-started from the walking policy; trains forward walking while wrist-yaw links stay near wheelchair handle targets. |
| [G1 dynamic wheelchair push](wheelchair-push-demo.md#dynamic-wheelchair-push-training) | `Unitree-G1-29dof-Wheelchair-Dynamic-Push` | Warm-started from the handle-grip policy; trains a passive wheelchair to move forward through handle contact while penalizing non-handle contact. |
| [G1 observed dynamic wheelchair push](wheelchair-push-demo.md#observed-dynamic-push-variant) | `Unitree-G1-29dof-Wheelchair-Dynamic-Push-Observed` | Active conservative adaptation run; adds wheelchair pose/handle observations to correct veering without reward-only guessing. |
| [G1 attached-hands wheelchair push](wheelchair-push-demo.md#attached-hands-variant) | `Unitree-G1-29dof-Wheelchair-Dynamic-Push-Attached` | Active follow-up; anchors the rubber hands to the wheelchair handles with spherical USD joints so the hands stay on the handles while wrist rotation remains free. |
| [G1 wheelchair standing bridge](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Dynamic-Stand-Observed-Neutral` | Completed bridge from plain standing into wheelchair-present standing before reintroducing handle pose and hand-handle attachment. |
| [G1 fixed-base wheelchair hold](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Fixed-Stand-Attached` | Stopped diagnostic run; it reduced early instability but risks teaching support from an immovable chair. |
| [G1 free-chair stationary hold attempts](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Stationary-Stand-Attached`, `Unitree-G1-29dof-Wheelchair-Braked-Stationary-Stand-Attached` | Stopped failed bridge attempts; free/braked stationary chair plus attached hands still collapsed from the neutral standing checkpoint. |
| [G1 full handle-arm standing bridge](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Stand-Handle-Arms` | Stopped; full handle pose with no chair was still too abrupt from the plain standing checkpoint. |
| [G1 partial reach-arm standing bridge](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Stand-Reach-Arms` | Stopped at `model_8400.pt`; used as a warm start for the immediate attached-wheelchair retry. |
| [G1 braked attached standing retry](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Braked-Stationary-Stand-Attached` | Stopped; actual free wheelchair plus hand-handle attachment was present, but the policy collapsed immediately from the reach-arm checkpoint. |
| [G1 relaxed attached standing retries](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Relaxed-Stand-Attached`, `Unitree-G1-29dof-Wheelchair-Left-Hand-Relaxed-Stand-Attached` | Stopped diagnostics; arm motion was restored and chair penalties were relaxed, but startup still collapsed with the hand-handle attachment active. |
| [G1 fixed-base relaxed standing retry](wheelchair-push-demo.md#standing-first-attached-pretrain) | `Unitree-G1-29dof-Wheelchair-Fixed-Relaxed-Stand-Attached` | Active diagnostic; locks the wheelchair root while keeping relaxed arm/wrist action so the robot can learn the hand-held standing pose without free-chair instability. |

New policy experiments should get their own page or subfolder here. Every variant should record its task ID, config file, command, checkpoint lineage, TensorBoard behavior, and playback notes.
