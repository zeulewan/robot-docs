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

New policy experiments should get their own page or subfolder here. Every variant should record its task ID, config file, command, checkpoint lineage, TensorBoard behavior, and playback notes.
