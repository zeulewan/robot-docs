# Unitree Simulation Stack

Unitree repos and tools for simulating, training, validating, and eventually deploying G1 policies.

**Guides:** [Developer Workflow](developer-workflow.md) · [RL Training Guide](rl-training-guide.md) · [TensorBoard - Training Monitoring](tensorboard.md) · [Sim-to-Real Deploy](deploy.md)

## Repo Overview

| Repo | What it is | Main role in this project |
|------|-----------|----------|
| **unitree_ros** | Robot model source files: URDFs, meshes, Xacro files, and older ROS/Gazebo packages. | Source for the G1 29DOF URDF used in training. |
| **unitree_model** | Preconverted Unitree 3D models, mainly USDs for Isaac/Omniverse. GitHub repo is deprecated in favor of Hugging Face. | Optional source for USD assets. Its README says the USDs come from URDF conversion. |
| **unitree_rl_lab** | Isaac Lab RL training framework for Unitree robots. | Train/play/export the G1 29DOF locomotion policy. |
| **unitree_sim_isaaclab** | Rich Isaac Lab/Isaac Sim application with DDS, sensors, teleop, replay, and task scenes. | Warehouse validation/control after training. |
| **unitree_mujoco** | Lightweight MuJoCo simulator built on SDK2. | Sim-to-real validation gate before hardware. |
| **unitree_sdk2 / unitree_sdk2_python** | Real robot C++/Python control SDKs over Unitree DDS/CycloneDDS. | Physical robot control path. |
| **unitree_ros2** | ROS 2 message/package layer for Unitree DDS topics. | ROS 2 integration with real or simulated Unitree DDS topics. |
| **xr_teleoperate / unitree_lerobot** | XR teleoperation and imitation-learning/data workflows. | Data collection and downstream learning for manipulation. |

### How they relate

```
unitree_ros or unitree_model
  -> canonical G1 29DOF robot asset
  -> unitree_rl_lab training
  -> exported policy
  -> unitree_sim_isaaclab warehouse validation/control
  -> unitree_mujoco / SDK2 / ROS 2 real-robot gates
```

Isaac Sim is the simulator. Isaac Lab is the robot-learning framework built on top of Isaac Sim. `unitree_rl_lab` and `unitree_sim_isaaclab` are Unitree projects built on Isaac Lab, but they have different jobs: `unitree_rl_lab` is the cleaner locomotion training surface, while `unitree_sim_isaaclab` is the richer validation/control/data app.

For the full repo-by-repo map, see [Developer Workflow](developer-workflow.md).

---

## unitree_rl_lab

**Purpose:** Train and evaluate RL locomotion policies for Unitree robots.

**Supported robots:** Go2, H1, G1-29DOF

**Key commands:**

```bash
# Train a G1 locomotion policy (headless, ~30 min on RTX 3090)
cd ~/GIT/unitree_rl_lab
python scripts/rsl_rl/train.py --headless --task Unitree-G1-29dof-Velocity

# Play back trained policy with GUI
python scripts/rsl_rl/play.py --task Unitree-G1-29dof-Velocity

# Monitor training (TensorBoard)
tensorboard --logdir logs/rsl_rl/ --host 0.0.0.0
# Then open http://workstation:6006
```

**Training details:**

- Runs 4096 parallel G1 robots on the GPU
- Uses PPO (Proximal Policy Optimization) via RSL-RL
- Checkpoints saved every 100 iterations to `logs/rsl_rl/unitree_g1_29dof_velocity/<timestamp>/`
- Produces both `.pt` (PyTorch) and `.onnx` (portable) policy files
- The `.onnx` gets deployed on the real G1's Jetson Orin

**What the locomotion policy does:**

```
Velocity command (vx, vy, yaw_rate)  →  Locomotion Policy (neural network)  →  29 joint actions
```

The policy converts a velocity command ("walk forward at 0.5 m/s") into the 29 joint positions/torques needed to walk without falling. Input source doesn't matter — keyboard, joystick, VR headset, or autonomous planner.

---

## unitree_sim_isaaclab

**Purpose:** Full simulation environment for sim-to-real transfer. Uses the same DDS communication protocol as the physical G1.

**Key features:**

- Manipulation tasks (pick-and-place, stacking) with G1 + dexterous hands
- DDS communication (same as real robot — code transfers directly)
- Pre-trained ONNX locomotion policy for walking (lower body)
- Teleoperation pipeline (VR headset, keyboard via DDS)
- Data collection and replay for imitation learning
- Camera streaming (for vision-based policies)

**Pre-trained models:** Downloaded via `fetch_assets.sh` from HuggingFace. These are Unitree-provided test assets, not our trained G1 policy.

**When to use:** Full sim-to-real pipeline with DDS, manipulation, or teleoperation. For locomotion training only, use `unitree_rl_lab`.

For our warehouse demo, `assets/model/our_policy.pt` was copied from the `unitree_rl_lab` training run, and the custom `Isaac-Locomotion-G129-Warehouse` task used a plain G1 29DOF USD generated from the same `unitree_ros` URDF used for training.

---

## unitree_ros

**Purpose:** URDF robot description files for all Unitree robots.

A URDF describes a robot's physical structure: links, joints, meshes, and physical properties such as mass and inertia. Isaac Sim can import URDFs directly or convert them into USD assets.

**G1 URDF variants available:**

| File | DOF | Description |
|------|-----|-------------|
| `g1_23dof.urdf` | 23 | Base G1 without wrist joints |
| `g1_29dof_rev_1_0.urdf` | 29 | Plain G1 29DOF Rev 1.0. This is the one used for our locomotion policy. |
| `g1_29dof_with_hand_rev_1_0.urdf` | 29+ | G1 with Inspire dexterous hands |
| `g1_29dof_lock_waist.urdf` | 29 | Waist locked (for arm-only tasks) |

---

## Installation

### Prerequisites

- Isaac Sim 5.1 (pip install in `isaaclab` conda env)
- Isaac Lab 2.3.2 (pip install in same env)

### Step 1: Clone repos

```bash
cd ~/GIT
git clone https://github.com/unitreerobotics/unitree_rl_lab.git
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git  # optional
```

### Step 2: Configure URDF path

Edit `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`:

```python
UNITREE_ROS_DIR = "/home/zeul/GIT/unitree_ros"
```

Switch the G1 29DOF config from USD to URDF spawner: uncomment `UnitreeUrdfFileCfg`, comment out `UnitreeUsdFileCfg`, and point it at `robots/g1_description/g1_29dof_rev_1_0.urdf`.

This is intentional for the current G1 29DOF velocity policy because `unitree_rl_lab` recommends URDF files for Isaac Sim 5.x and the selected URDF exactly matches the plain 29DOF task.

### Step 3: Install unitree_rl_lab

```bash
conda activate isaaclab
cd ~/GIT/unitree_rl_lab
pip install -e "source/unitree_rl_lab"
```

### Step 4 (optional): Install unitree_sim_isaaclab dependencies

```bash
conda activate isaaclab
pip install -r ~/GIT/unitree_sim_isaaclab/requirements.txt
# Fix cryptography version conflict:
pip install cryptography==44.0.0
```

Download assets (USD models + pre-trained ONNX policy):

```bash
cd ~/GIT/unitree_sim_isaaclab
sudo apt install -y git-lfs  # if not already installed
bash fetch_assets.sh  # downloads ~1.2 GB from HuggingFace
```

---

## Key Concepts

### Policy formats

| Format | Extension | Used for | Can resume training? |
|--------|-----------|----------|---------------------|
| PyTorch checkpoint | `.pt` | Sim playback, fine-tuning | Yes |
| ONNX | `.onnx` | Real robot deployment, portable inference | No |

Both contain the same neural network. ONNX is a portable export format (like PDF for documents). PyTorch is the native training format.

### DDS (Data Distribution Service)

The networking layer under ROS 2. Normally invisible. Matters here because:

- **Isaac Sim host** uses FastDDS (default in ROS 2 Jazzy)
- **G1 robot** uses CycloneDDS
- **unitree_sim_isaaclab** uses DDS directly (bypasses ROS 2 for lower latency)
- Host-to-container communication requires FastDDS shared memory disabled (UDP-only)

### URDF vs USD

| | URDF | USD |
|---|---|---|
| Format | XML text + mesh files | Binary 3D scene (Pixar/NVIDIA) |
| Origin | ROS ecosystem | Omniverse / film industry |
| Contains | Robot structure only | Full scene with materials |
| Standard in | Robotics | NVIDIA simulation |

Isaac Sim supports both. We used URDF for training because it matched the Unitree G1 29DOF velocity task cleanly. We used USD for the warehouse validation scene because Isaac Sim worlds, sensors, cameras, lidar, and scene assets are USD-native.

The critical rule is asset/interface parity: same robot variant, same joint names/order, same action count, same default pose, same actuator setup, same observation shape, and compatible physics settings.
