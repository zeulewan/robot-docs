# Unitree Developer Workflow

This page explains how the Unitree repositories fit together, what file formats are involved, and what workflow should be treated as canonical for the G1 29DOF locomotion project.

## Short Version

The intended workflow for this project is:

```text
unitree_ros URDF/meshes or official Unitree model source
  -> canonical G1 29DOF robot asset
  -> unitree_rl_lab training
  -> exported policy
  -> unitree_sim_isaaclab validation/control
  -> unitree_mujoco and real robot gates
```

What we actually did matches that shape, minus the MuJoCo and real-robot gates. The weak spot is that the canonical asset step was discovered manually while debugging. Going forward, the robot asset source, conversion settings, joint order, actuator setup, observation shape, action scale, and physics settings should be pinned and checked explicitly.

## Repository Role Map

| Repository | What it is | Use it for |
|---|---|---|
| [`unitree_ros`](https://github.com/unitreerobotics/unitree_ros) | Broad robot-description source repo with URDFs, meshes, Xacro files, and older ROS/Gazebo packages. | Source robot geometry, joints, masses, inertias, and mesh references. For this project, this is where the G1 29DOF URDF came from. |
| [`unitree_model`](https://github.com/unitreerobotics/unitree_model) | Preconverted Unitree robot model repo/dataset. GitHub repo is deprecated in favor of Hugging Face. | USD assets for Isaac/Omniverse workflows. Its README says the USDs are generated from URDF. |
| [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) | Core C++ SDK for newer Unitree robots. | Real robot communication and control over Unitree's DDS/CycloneDDS stack. |
| [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python) | Python binding for SDK2. | Python DDS control clients, examples, and quick integration scripts. |
| [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) | ROS 2 package and message layer for Unitree robots. | ROS 2 integration using the same DDS substrate as SDK2. Topics like low state, low command, sport mode state, wireless controller, and API request/response live here. |
| [`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) | Lightweight simulator built on Unitree SDK2 and MuJoCo. | Sim-to-real validation for controllers written against SDK2, SDK2 Python, or ROS 2 before running on the physical robot. |
| [`unitree_rl_lab`](https://github.com/unitreerobotics/unitree_rl_lab) | Isaac Lab reinforcement learning repo for Unitree robots. | Training and playing locomotion or imitation policies in Isaac Lab. This is where our G1 29DOF velocity policy was trained. |
| [`unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab) | MuJoCo-based RL training repo using an Isaac-Lab-like API. | Lighter/faster locomotion RL. Its README presents the workflow as `Train -> Play -> Sim2Real`. |
| [`unitree_sim_isaaclab`](https://github.com/unitreerobotics/unitree_sim_isaaclab) | Rich Isaac Lab/Isaac Sim application for Unitree tasks. | Scene-level validation, DDS-style control, teleoperation, sensors, replay, data generation, and model validation. Not the cleanest primary locomotion training repo as-is. |
| [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) | XR teleoperation stack for Unitree humanoids. | Collecting teleoperation data, including simulation workflows that launch `unitree_sim_isaaclab`. |
| [`unitree_lerobot`](https://github.com/unitreerobotics/unitree_lerobot) | Unitree's LeRobot/imitation-learning integration. | Training/testing policies from collected Unitree robot data, especially G1 dexterous-hand datasets. |

## File Formats

| Format | Stands for | What it is |
|---|---|---|
| URDF | Unified Robot Description Format | ROS XML format for robot links, joints, inertias, limits, and mesh references. Good robot source file. |
| USD | Universal Scene Description | Pixar/NVIDIA Omniverse scene format used heavily by Isaac Sim for robots, worlds, sensors, materials, and simulation scenes. |
| MJCF | MuJoCo XML format | MuJoCo's native model format. Used by `unitree_mujoco` and MuJoCo-based training/validation. |
| `.pt` | PyTorch/JIT policy | Native training/checkpoint or TorchScript export format. Useful for Isaac Lab playback when normalizers are embedded. |
| `.onnx` | Open Neural Network Exchange | Portable inference format for deployment on other runtimes, including real robot controllers. |

URDF and USD are not competitors in a simple "one is better" sense. URDF is a compact robot-description format. USD is Isaac/Omniverse's native scene/asset format. Isaac can use either, but USD is usually better for complete scenes, materials, cameras, lidar, and prebuilt simulation assets.

## Why We Used `unitree_ros`

`unitree_rl_lab` explicitly supports two robot-description paths: USD assets from Unitree's model dataset, or URDF files from `unitree_ros`. Its README recommends the URDF path for Isaac Sim 5.x.

For the G1 walking policy, we used:

```text
unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf
```

That file matched the training task `Unitree-G1-29dof-Velocity`: plain G1, 29 controllable DOFs, no dexterous hands, no base-fixed manipulation setup, and no locked-waist variant. The other G1 files are valid, but they describe different robots or task assumptions, such as 23DOF, hand-equipped, waist-locked, or manipulation-oriented variants.

## Training vs Validation

Training and validation had different jobs:

| Stage | Repo | Asset used | Purpose |
|---|---|---|---|
| RL training | `unitree_rl_lab` | G1 29DOF URDF from `unitree_ros` | Learn a velocity-tracking locomotion policy with Unitree's default Isaac Lab task, rewards, observations, and actions. |
| Simple playback | `unitree_rl_lab` | Same training task/config | Check that the trained checkpoint walks in the same Isaac Lab environment. |
| Warehouse validation/control | `unitree_sim_isaaclab` | Matching G1 29DOF USD | Run the trained policy inside a richer Isaac Sim scene with DDS-style command input, camera, lidar, and bridges. |
| Sim-to-real gate | `unitree_mujoco` | MuJoCo/MJCF robot | Validate SDK2/ROS2-style controllers before touching hardware. Not yet completed for this project. |
| Real robot | `unitree_sdk2`, `unitree_sdk2_python`, or `unitree_ros2` | Physical G1 | Run the policy/controller over Unitree DDS/CycloneDDS. Not yet completed for this project. |

The policy itself does not contain a URDF or USD. It is a neural network that expects a specific observation vector and outputs 29 action targets. That is why the robot asset and task interface must match between training and playback.

## Our Actual G1 Locomotion Run

The main training task was:

```text
Unitree-G1-29dof-Velocity
```

The main training run was:

```text
unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/
```

The final checkpoint was `model_7200.pt`. The exported policy files were `exported/policy.pt` and `exported/policy.onnx`. The policy uses Unitree's default G1 29DOF velocity environment: velocity-command tracking, uprightness, base height, gait rhythm, foot clearance, joint-limit penalties, action smoothness, energy penalty, and contact/fall termination logic.

For the warehouse demo, `unitree_sim_isaaclab/assets/model/our_policy.pt` was copied from that exported JIT policy. The custom task was:

```text
Isaac-Locomotion-G129-Warehouse
```

The matching plain locomotion USD was:

```text
unitree_sim_isaaclab/assets/robots/g1-29dof-locomotion/g1_29dof_rev_1_0.usd
```

Its conversion metadata shows it was generated from:

```text
/home/zeul/GIT/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf
```

That is the important link: the working warehouse robot USD came from the same G1 29DOF URDF lineage as the training robot.

## Why the First USDs Were Risky

`unitree_sim_isaaclab` ships useful USD assets, but many of the G1 USDs are for dexterous hands, base-fixed manipulation, whole-body manipulation, or other task variants. A robot can look like a G1 and still be wrong for a locomotion policy if the policy expects a different joint set or action interface.

Things that must match:

| Interface item | Why it matters |
|---|---|
| Joint names and order | The policy outputs 29 actions. The action manager must apply them to the intended joints. |
| DOF count | A 23DOF, hand-equipped, or locked-waist variant changes the action/observation contract. |
| Default pose | Observations include relative joint positions around the default pose. |
| Actuator gains and limits | The same action can produce different motion with different stiffness, damping, effort limits, or velocity limits. |
| Observation history and scaling | The trained policy expects the same observation vector shape and normalization. |
| Decimation and timestep | Policy frequency and physics stepping affect stability. |
| Collision and solver settings | URDF-to-USD conversion can change self-collision, convex hulls, fixed-joint merging, and PhysX solver iterations. |

This is why the final action-provider approach was important. `CustomRLActionProviderV2` does not manually rebuild the 480-dimensional observation vector or manually map actions. It uses Isaac Lab's `ObservationManager` and `ActionManager`, preserving the same training pipeline inside `unitree_sim_isaaclab`.

## Recommended Project Workflow

For future work, treat this as the preferred workflow:

1. Pick the exact robot variant first. For this project, that is plain G1 29DOF Rev 1.0.
2. Pin the source model repository and commit, usually `unitree_ros` for URDF/meshes or the official Unitree model dataset for USDs.
3. Generate one canonical USD from the selected URDF if Isaac Sim scene work needs USD.
4. Record the conversion settings: movable base, drive type, stiffness/damping, self-collision, collider type, fixed-joint merge behavior, solver settings, and sensor additions.
5. Train in `unitree_rl_lab` or a clean Isaac Lab extension using the canonical robot interface.
6. Export the policy as both JIT `.pt` and ONNX.
7. Validate in `unitree_sim_isaaclab` with a manager-based policy provider, DDS command path, cameras, lidar, and the warehouse scene.
8. Add a MuJoCo sim-to-real gate with `unitree_mujoco` before real hardware.
9. Only then run through SDK2/ROS2 on the physical G1.

Training directly on USD is possible and may be cleaner if the USD is the canonical asset used for playback and validation. Training on a random available USD is not safe. The requirement is asset/interface parity, not the file extension.

## Parity Checklist

Before trusting a trained policy outside its original training environment, check:

| Check | Expected for this project |
|---|---|
| Robot variant | Plain G1 29DOF Rev 1.0 |
| Source robot file | `g1_29dof_rev_1_0.urdf` from `unitree_ros` |
| Action count | 29 |
| Observation shape | Same as `Unitree-G1-29dof-Velocity` training config |
| Joint names/order | Same action manager ordering as training |
| Default pose | Same standing pose from the training config |
| Actuators | Same stiffness, damping, effort limits, velocity limits, and armature |
| Physics timing | Same policy decimation and timestep |
| Collision settings | Reviewed after URDF-to-USD conversion |
| Policy file | Exported from the intended training run, not Unitree's default pretrained asset |

## Maintenance Snapshot

As of May 5, 2026, the repos most relevant to the modern G1 workflow are still active. GitHub metadata showed pushed activity in 2026 for `unitree_ros`, `unitree_model`, `unitree_sdk2`, `unitree_sdk2_python`, `unitree_ros2`, `unitree_rl_lab`, `unitree_rl_mjlab`, `unitree_sim_isaaclab`, `xr_teleoperate`, and `unitree_lerobot`. `unitree_mujoco` had older pushed code in the observed metadata, but it remains the official lightweight SDK2/MuJoCo validation repo. The local workstation clones are not all equally fresh: `unitree_ros` and `unitree_sim_isaaclab` were behind their remotes during inspection, while `unitree_rl_lab` matched its observed remote head.

Do not use "most recently updated" as the only signal for which repo to use. The repos have different jobs. For locomotion RL, start with `unitree_rl_lab` or `unitree_rl_mjlab`. For rich Isaac Sim scene validation, use `unitree_sim_isaaclab`. For real robot control, use SDK2/ROS2. For robot descriptions, use `unitree_ros` or Unitree's model dataset.
