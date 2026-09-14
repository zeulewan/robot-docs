# Manraj Handoff

This is the starting point for continuing the Unitree G1, Vicon, Isaac Lab, and
teleoperation work.

## Read First

1. [G1 Current State](../infrastructure/unitree-g1/current-state.md) -- the
   physical robot is not cleared for actuation because five left-arm joints were
   missing normal telemetry at the last check.
2. [Network Topology](../infrastructure/networking/topology.md) -- local SSH
   routes, field router, robot router, and Vicon Windows host.
3. [Vicon LAN](../infrastructure/networking/vicon-lan.md) -- Nexus and camera
   bring-up on the dedicated Windows computer.
4. [Unitree Simulation Stack](../simulation/unitree/index.md) -- repository and
   simulation map.

Credentials are intentionally absent from the website and Git repository. Use
the private robot infrastructure secrets file supplied directly by Zeul.

## Systems

| System | Primary role | Current direction |
|---|---|---|
| RTX 3090 workstation | Main Isaac Lab, CloudXR, RL, and MuJoCo host | Quest simulation and policy work |
| Vicon Windows / RTX 4060 | Nexus capture and secondary native-Windows Isaac Lab | Mocap plus small RL smoke tests |
| `jeffxi-ubuntu` | Home Assistant and lab support services | Power and network operations |
| G1 Jetson | Local DDS, cameras, and future XR bridge | Read-only checks until hardware gate passes |
| G1 locomotion board | Unitree motion services | No SSH; communicate through SDK2/DDS |

## Active Workstreams

### Quest teleoperation

[Meta Quest 3 Teleoperation](../simulation/unitree/quest-3-teleoperation.md)
documents the working NVIDIA Isaac Teleop and CloudXR simulation route, the
wheelchair scene, the older Unitree/Vuer experiment, and the gated real-robot
path.

### GEAR-SONIC

[GEAR-SONIC](../simulation/unitree/gear-sonic.md) documents the released NVIDIA
whole-body controller, known-good MuJoCo startup, and the local uncommitted Vicon
trajectory adapters. Preserve that dirty checkout before updating it.

### Unitree RL Lab and wheelchair pushing

[RL Training](../simulation/unitree/rl-training-guide.md) covers locomotion
training. [Wheelchair Push](../simulation/unitree/policy-experiments/wheelchair-push/index.md)
records the contact model, curriculum, policy experiments, and articulated CC0
wheelchair asset.

### Dex3 manipulation

[Dex3 ACT Manipulation](../simulation/unitree/dex3-act.md) records the verified
single-policy three-block stacking result and the LeRobot data/training path.

### Vicon and Windows Isaac Lab

Use official Nexus for the cameras. [Isaac Lab on Vicon Windows](../simulation/unitree/windows-isaac-lab.md)
documents the separate native-Windows Unitree RL Lab installation. WSL is not
the Isaac Sim runtime.

## Safe Default

Simulation is the default. Before any physical robot session, require support
frame placement, a clear workspace, normal telemetry from every arm and hand,
an R3 stop operator, a tested software stop, and explicit session authorization.
Do not start a launcher merely to see whether it connects; several launchers
create motor publishers during initialization.

## Before Changing Repositories

Run `git status --short --branch` and preserve local work. In particular,
`GR00T-WholeBodyControl`, `unitree_rl_lab`, `xr_teleoperate`, and
`IsaacLab-current` contain project-specific changes or experiments. Do not clean,
reset, pull, or rebase them as a first step.
