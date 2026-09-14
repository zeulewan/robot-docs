# Isaac Lab on the Vicon Windows Host

The physical Vicon computer is also a secondary native-Windows Isaac Lab host.
This setup is for Manraj's local development and small Unitree RL Lab tests. It
does not replace the RTX 3090 workstation for larger training runs.

## Verified Installation

Verified 2026-08-27 on `OMG-TMU-DAE`:

| Component | Version or state |
|---|---|
| GPU | NVIDIA RTX 4060 Laptop GPU, 8 GB VRAM |
| Windows NVIDIA driver | `560.94` |
| Python | `3.11.16` in Conda environment `env_isaaclab` |
| Isaac Sim | `5.1.0.0` |
| Isaac Lab | `2.3.2` |
| PyTorch | `2.7` with CUDA 12.8 support |
| RSL-RL | `3.1.2` |
| Unitree RL Lab | Official GitHub checkout, commit beginning `4960b847` |
| Unitree ROS | Official GitHub checkout, commit beginning `80b964` |

A native Windows smoke test launched `Unitree-G1-29dof-Velocity` on CUDA with
four environments for one training iteration. It completed 96 timesteps and
wrote `model_0.pt`. A later rendering test used Direct3D 12 plus Vulkan and
completed successfully at about 42 simulation steps per second.

## Use Native Windows

Isaac Sim and Isaac Lab run on **native Windows** on this computer. WSL can see
CUDA through `/dev/dxg`, but it did not provide the Vulkan graphics path that
Isaac Sim requires. Use WSL for shell utilities or ROS-side work, not for Isaac
Sim rendering or PhysX GPU simulation.

Do not install a Linux NVIDIA display driver inside WSL. Its GPU access is
provided by the Windows host driver.

## Local Handoff Files

When the host is online, read these files from Manraj's Windows profile before
changing the environment:

```text
C:\Users\manraj.OMG-TMU-DAE\UNITREE_RL_LAB_AGENT_HANDOFF.md
C:\Users\manraj.OMG-TMU-DAE\UNITREE_RL_LAB_SETUP.md
C:\Users\manraj.OMG-TMU-DAE\unitree-env.cmd
```

The launcher opens the configured Conda environment and repository. Preserve the
working smoke-test environment before upgrading Isaac, CUDA, the NVIDIA driver,
or the Unitree repositories.

## Scope and Limits

- The RTX 4060 is appropriate for installation checks, policy playback, and
  small-environment training tests.
- Use the RTX 3090 workstation for the established 4096-environment locomotion
  runs and the Quest/CloudXR stack.
- Nexus camera capture and Isaac Lab are separate applications. Close unneeded
  GPU workloads before relying on interactive performance.
- No physical G1 deployment should be started from this host while the current
  left-arm telemetry fault remains unresolved.

## Connectivity

The Windows host's local lab address is `192.168.8.132`. Manraj's separate
Tailscale account can reach the Vicon host remotely, then use it as an SSH jump
host to local lab systems. The host does not advertise the whole lab subnet and
is not an exit node.

See [Vicon LAN](../../infrastructure/networking/vicon-lan.md) for the camera
network and [Network Topology](../../infrastructure/networking/topology.md) for
the lab access map.
