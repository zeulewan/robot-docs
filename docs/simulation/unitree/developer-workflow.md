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
| [`unitree_model`](https://huggingface.co/datasets/unitreerobotics/unitree_model) | Preconverted Unitree robot model dataset on Hugging Face. The old GitHub `unitree_model` repo is deprecated in favor of Hugging Face. | Current official source for Unitree-provided USD assets for Isaac/Omniverse workflows. Its card says to visit `unitree_ros` for more robot-model information and describes URDF-to-USD conversion settings. |
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

## Which Source Is Truth?

Use different "truth" sources for different layers:

| Layer | Preferred source | Why |
|---|---|---|
| Robot mechanics and URDF description | `unitree_ros` on GitHub | Contains the URDFs, meshes, links, joints, inertias, limits, and mount frames. This is the source used for our G1 29DOF training run. |
| Unitree-provided USD assets | `unitree_model` on Hugging Face | The old GitHub `unitree_model` repo says future updates are released on Hugging Face. Use HF for current preconverted USDs and USD sensor/physics layers. |
| Task code, training code, DDS simulation app | GitHub repos such as `unitree_rl_lab` and `unitree_sim_isaaclab` | These are normal source-code repos, not large binary asset stores. |
| Large robot datasets and model checkpoints | Unitree Hugging Face org | Hugging Face is where Unitree publishes embodied-AI datasets, UnifoLM models, and large downloadable assets. |

For this project, the G1 locomotion training source of truth was still `unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf`. Hugging Face does not replace that URDF source for the training run. Hugging Face does supersede the old `unitree_model` GitHub repo for Unitree's preconverted USD assets, so it should be checked when choosing or comparing Isaac/Omniverse robot USDs.

## Why We Used `unitree_ros`

`unitree_rl_lab` explicitly supports two robot-description paths: USD assets from Unitree's model dataset, or URDF files from `unitree_ros`. Its README recommends the URDF path for Isaac Sim 5.x.

For the G1 walking policy, we used:

```text
unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf
```

That file matched the training task `Unitree-G1-29dof-Velocity`: plain G1, 29 controllable DOFs, no dexterous hands, no base-fixed manipulation setup, and no locked-waist variant. The other G1 files are valid, but they describe different robots or task assumptions, such as 23DOF, hand-equipped, waist-locked, or manipulation-oriented variants.

## Sensor Frames vs Active Sensors

Treat the URDF as the source of truth for robot mechanics and mount frames, not as the source of truth for active Isaac sensors. The G1 URDF contains links such as `imu_in_torso`, `imu_in_pelvis`, and `d435_link`. Those are useful frames: they tell code where an IMU or camera would be mounted relative to the robot body. They do not automatically publish images, point clouds, or IMU messages inside Isaac Lab.

In USD/Isaac Sim, a `prim` is a scene node addressed by a path, similar to a filesystem path. Examples:

```text
/World/envs/env_0/Robot/torso_link
/World/envs/env_0/Robot/d435_link
/World/envs/env_0/Robot/d435_link/front_cam
```

The first two paths can be passive robot/link prims. The last path can be an active camera prim if a task config creates one there.

`unitree_sim_isaaclab` usually creates active cameras from task configs, not from the URDF alone. The reusable helpers live here:

```text
unitree_sim_isaaclab/tasks/common_config/camera_configs.py
```

A task uses those helpers by adding camera fields to its scene config:

```python
front_camera = CameraPresets.g1_front_camera()
left_wrist_camera = CameraPresets.left_gripper_wrist_camera()
right_wrist_camera = CameraPresets.right_gripper_wrist_camera()
```

`CameraPresets.g1_front_camera()` returns an Isaac Lab `CameraCfg` with a default prim path under `Robot/d435_link/front_cam`, a `PinholeCameraCfg` spawn config, image size, update period, and camera offset. When Isaac Lab builds the environment, it sees the `front_camera` field, spawns the camera prim in the USD stage, and registers the runtime sensor as `env.scene["front_camera"]`. Observation or bridge code then reads rendered data from `env.scene["front_camera"].data.output["rgb"]`.

Those camera helpers only configure cameras. They do not configure IMU or lidar. In the warehouse locomotion task, the head camera was defined directly as `head_camera = CameraCfg(...)` in the task scene config instead of using the common preset helper. The RTX lidar used for the warehouse bridge was a separate USD/sensor addition, and the IMU stream was derived from robot simulation state rather than created by the camera helper.

## SLAM and Visualization Split

The RGB-D RTAB-Map test path deliberately separates SLAM inputs from operator visualization.

RTAB-Map consumes raw RGB-D data:

```text
/head_camera/color/image_raw
/head_camera/depth/image_raw
/head_camera/color/camera_info
```

The operator should normally view the compressed preview and lightweight RTAB outputs:

```text
/head_camera/image/compressed
/rtabmap/map
/rtabmap/grid_prob_map
/rtabmap/odom
/rtabmap/mapPath
```

Do not use Foxglove-over-network as the default renderer for the full colored global point cloud. `/rtabmap/cloud_map` is an accumulated `PointCloud2`; in the warehouse test it measured about 12 MB per message. Foxglove must move that raw message over WebSocket, deserialize it in the browser, and render it in WebGL. Moonlight is different: the workstation renders the 3D view locally, then sends a compressed video stream. That is why Isaac Sim can look smooth over Moonlight while a browser-based colored RTAB cloud map stutters.

Use Foxglove for driving, preview, commands, odometry, path, and 2D/occupancy-style map views. Use RViz2 or `rtabmap_viz` on the workstation, viewed through Moonlight, when the full colored 3D point cloud is needed. The current ROS container has RViz2 and `rtabmap_viz` installed, but Docker display wiring still needs to be added before GUI tools can launch directly from inside that container.

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
