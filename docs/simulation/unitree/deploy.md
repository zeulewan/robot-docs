# Sim-to-Real Deployment

Deploying a trained G1-29DOF locomotion policy to the physical robot.

## Overview

The deploy pipeline runs a C++ controller (`g1_ctrl`) on a laptop directly wired to the robot. The controller:

1. Loads the trained ONNX policy
2. Reads joint state + IMU from the robot via DDS (CycloneDDS over ethernet)
3. Runs the policy at 50 Hz
4. Sends joint position commands back to the robot

The controller runs on the **Mac** (or any machine with a direct ethernet connection to the G1). It does **not** run on the Jetson.

```
Mac (g1_ctrl) ←─── DDS over ethernet ───→ G1 (locomotion computer)
    ↑
policy.onnx (ONNX Runtime)
```

## Our Trained Policy

Location on workstation:

```
~/GIT/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46/
├── exported/
│   ├── policy.onnx   ← deploy this
│   └── policy.pt
├── params/
│   └── deploy.yaml   ← PD gains, joint IDs, obs scaling
└── model_7200.pt     ← final checkpoint (7200 epochs)
```

**Velocity ranges trained:**

| Axis | Min | Max |
|------|-----|-----|
| Forward (vx) | -0.5 m/s | 1.0 m/s |
| Lateral (vy) | -0.3 m/s | 0.3 m/s |
| Yaw rate | -0.2 rad/s | 0.2 rad/s |

## Setup: Build the Controller

### Prerequisites (install once on workstation or Mac)

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

### Install unitree_sdk2

```bash
git clone git@github.com:unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

This installs the DDS library to `/usr/local`.

### Compile g1_ctrl

```bash
cd ~/GIT/unitree_rl_lab/deploy/robots/g1_29dof
mkdir build && cd build
cmake .. && make
```

Binary: `deploy/robots/g1_29dof/build/g1_ctrl`

## Configure: Point at Our Trained Policy

Edit `deploy/robots/g1_29dof/config/config.yaml`, find the `Velocity` section and switch the `policy_dir`:

```yaml
Velocity:
  # policy_dir: config/policy/velocity  # ← default (Unitree's pre-trained)
  policy_dir: ../../../logs/rsl_rl/unitree_g1_29dof_velocity/2026-03-06_14-30-46  # ← our trained policy
```

The controller looks for `policy_dir/exported/policy.onnx` and `policy_dir/params/deploy.yaml`.

## Run: Connecting to the Robot

### Step 1: Wire up

On the Mac, connect USB-A to USB-C ethernet adapter to neck port 4 or 5. Set static IP:

```bash
sudo ifconfig en13 192.168.123.100 netmask 255.255.255.0 up
```

Verify Jetson is reachable:

```bash
ssh unitree@192.168.123.164
```

### Step 2: Close the onboard controller

!!! warning "Critical — do this before running g1_ctrl"
    The onboard locomotion program must be stopped before you send joint commands. Running two controllers simultaneously will cause undefined behavior and could damage the robot.

    Check with Unitree docs how to stop the onboard controller. Typically via the app or SSH to the locomotion computer (requires UniPwn or Unitree support).

### Step 3: Run g1_ctrl

```bash
cd ~/GIT/unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl --network en13   # en13 = USB ethernet adapter on Mac
```

### Step 4: Controller sequence

Use the wireless controller:

| Button | Action |
|--------|--------|
| `LT + ↑` | Passive → FixStand (robot stands up) |
| `RB + X` | FixStand → Velocity (RL policy active) |
| `LT + B` | Any → Passive (safe stop) |

Start slow — use small velocity commands to verify the policy behaves as expected before pushing to full speed.

## Troubleshooting

**DDS can't connect:**
Make sure the ethernet interface name matches (`--network en13` or whatever `ifconfig` shows). The G1's DDS domain is 0.

**Policy file not found:**
Check that `policy_dir` in `config.yaml` resolves correctly relative to the `g1_ctrl` binary's working directory. Run from the `build/` directory.

**Robot falls immediately:**
The trained policy has conservative velocity ranges. Do not command velocities outside the training range (±0.5 m/s forward, ±0.3 m/s lateral, ±0.2 rad/s yaw). Make sure FixStand completes fully (3-second interpolation) before switching to Velocity mode.
