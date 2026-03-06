# Unitree Simulation Stack

Unitree repos and tools for simulating the G1 in Isaac Sim.

**Guides:** [TensorBoard — Training Monitoring](tensorboard.md)

## Repo Overview

| Repo | What it is | Location |
|------|-----------|----------|
| **unitree_rl_lab** | RL training framework for Unitree robots. Train locomotion policies, play them back in sim. Built on Isaac Lab. | `~/GIT/unitree_rl_lab/` |
| **unitree_sim_isaaclab** | Unitree's full sim environment for teleoperation, data collection, and manipulation tasks. Uses DDS (same protocol as real robot). | `~/GIT/unitree_sim_isaaclab/` |
| **unitree_ros** | URDF robot description files — meshes, joint definitions, physical properties for all Unitree robots. | `~/GIT/unitree_ros/` |

### How they relate

```
Isaac Sim 5.1 (the rendering/physics engine)
  └── Isaac Lab 2.3.2 (the robotics RL framework)
       ├── unitree_rl_lab (RL training — locomotion, velocity control)
       └── unitree_sim_isaaclab (full sim — manipulation, teleoperation, DDS)

unitree_ros (robot URDF files — used by both)
```

Both Unitree repos are applications built on Isaac Lab. They share the same conda environment (`isaaclab`) and the same Isaac Sim installation.

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

**Pre-trained models:** Downloaded via `fetch_assets.sh` from HuggingFace (~1.2 GB). Located at `assets/model/policy.onnx`.

**When to use:** Full sim-to-real pipeline with DDS, manipulation, or teleoperation. For locomotion training only, use `unitree_rl_lab`.

---

## unitree_ros

**Purpose:** URDF robot description files for all Unitree robots.

A URDF describes a robot's physical structure — links, joints, meshes, and physical properties (mass, inertia). Isaac Sim reads the URDF to build the robot in simulation.

**G1 URDF variants available:**

| File | DOF | Description |
|------|-----|-------------|
| `g1_23dof.urdf` | 23 | Base G1 without wrist joints |
| `g1_29dof_rev_1_0.urdf` | 29 | Full G1 with wrist joints (recommended) |
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

Switch the G1 29DOF config from USD to URDF spawner — uncomment `UnitreeUrdfFileCfg`, comment out `UnitreeUsdFileCfg`.

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

Isaac Sim supports both. We use URDF because Unitree provides their robots as URDF.
