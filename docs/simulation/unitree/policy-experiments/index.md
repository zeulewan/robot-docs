# Policy Experiments

This folder tracks policy experiments separately from the base Unitree training guides. Each subfolder should describe one policy task, what it changed from the upstream task, how it was trained, and what behavior was observed.

## Active / Current

| Experiment | Task IDs | Status |
|---|---|---|
| [G1 running policy family](g1-running/index.md) | Multiple | Overview of the walking-to-running-to-sprint lineage. |
| [G1 running](g1-running/running.md) | `Unitree-G1-29dof-Running` | Completed first running pass; checkpoint lineage ends at `model_2100.pt`. |
| [G1 fast running](g1-running/fast-running.md) | `Unitree-G1-29dof-Running-Fast` | Completed fast pass to `model_5099.pt`; used as the sprint warm-start. |
| [G1 sprint 10 m/s](g1-running/sprint-10ms.md) | `Unitree-G1-29dof-Sprint-10ms` | Paused at `model_20500.pt`; curriculum reached about `7.2 m/s`, gait needs tuning. |
| [G1 sprint gait cleanup](g1-running/sprint-10ms.md#gait-cleanup-variant) | `Unitree-G1-29dof-Sprint-10ms-Gait` | New stability-gated sprint variant for improving the running gait. |
| [G1 wheelchair push policy](wheelchair-push/index.md) | Multiple | Active wheelchair-push work. May 19 audit shows the 2 m/s hard-attach checkpoint is a real visual reference, but hard hand-handle PhysX constraints produce catastrophic outliers at training scale. |

## Wheelchair Archive

The wheelchair work has many stopped variants, startup diagnostics, and old video captures. Those are intentionally kept out of the main experiment index now.

| Archive | Contents |
|---|---|
| [Wheelchair goal spec](wheelchair-push/spec.md) | Deliverable contract, milestone ladder, acceptance criteria, and allowed experiment knobs for the wheelchair project. |
| [Wheelchair chronological archive](wheelchair-push/archive.md) | Original full log with old commands, checkpoint paths, asset turntables, startup/ragdoll clips, X-rail experiments, and PhysX rail stabilization notes. |
| [Old compatibility page](wheelchair-push-demo.md) | Short pointer kept for links that still target the former wheelchair page. |

New policy experiments should get their own page or subfolder here. Every variant should record its task ID, config file, command, checkpoint lineage, TensorBoard behavior, and playback notes.
