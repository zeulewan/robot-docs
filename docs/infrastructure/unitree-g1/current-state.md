# Unitree G1 Current State

Last verified: **2026-09-02**. This is the operator-facing snapshot for the lab
G1. Recheck it on the robot before relying on it; powered hardware state can
change between sessions.

!!! danger "Do not actuate the robot yet"
    The last passive state check found missing telemetry on five left-arm joints.
    Keep the robot in its support frame and do not start a motor-command publisher
    until the left-arm power and communication path has been inspected with power
    removed, then verified through a read-only probe.

## Last Verified Robot State

| Check | Result |
|---|---|
| Motion mode | `ai` |
| Locomotion FSM | `0` (zero torque) |
| External arm command publisher | None detected |
| Arm command gains/torques | `kp=0`, `kd=0`, `tau=0` |
| Custom lab services | Disabled and inactive |
| Custom ROS, Isaac, or container workload | None detected |
| Jetson GPU workload | Idle apart from normal system use |

Right-arm telemetry and the first two left shoulder joints looked normal. Left
arm joint indices `17-21` instead reported `motorstate=0x80000000` and did not
provide normal position, temperature, or torque telemetry. This points to a
hardware power, connector, or communication problem rather than a software
controller holding the arm in damping mode.

The three lab-created services are intentionally stopped:

```text
g1-quest-webxr-probe.service
g1-teleimager.service
g1-quest-camera-preview.service
```

They were disabled with `systemctl disable --now` and verified inactive. Do not
reenable them merely to test connectivity; start only the one needed for the
current task and stop it afterward.

## Network and Access

| Component | Address | Purpose |
|---|---|---|
| Field router `eph107` | `192.168.8.1` | School uplink and lab management LAN |
| Robot router WG827 | `192.168.8.190` upstream, `192.168.123.1` robot LAN | Routes the private robot LAN |
| G1 Jetson | `192.168.123.164` | SDK2, cameras, and future teleoperation host |
| Locomotion board | `192.168.123.161` | Unitree low-level DDS services; SSH unavailable |
| Livox Mid-360 | `192.168.123.20` | Lidar |
| Ubuntu operator host | `192.168.8.241` | Home Assistant and lab support services |

The Jetson is reached through the robot router. Manraj's local SSH route is:

```bash
ssh -J root@192.168.8.190 manraj@192.168.123.164
```

Credentials and private key details are not stored in this repository. They are
in the local robot infrastructure secrets file supplied separately.

## Robot Power

The iDevices outlet controlled by Home Assistant is the **main robot power
switch**, not a convenience outlet.

| Item | Value |
|---|---|
| Home Assistant host | `http://192.168.8.241:8123/` |
| Power entity | `switch.switch_00101614` |

Before powering off, stop any motor-control process and let the robot reach a
stable supported state. After powering on, use passive checks before launching a
controller.

## Safe Bring-Up Order

1. Put the G1 on the support frame and clear the workspace.
2. With robot power off, inspect and reseat the left-arm power and communication
   connectors using the Unitree service procedure.
3. Power on and connect to the Jetson through `192.168.8.190`.
4. Confirm no custom controller is running.
5. Run the subscribe-only state probe from `/home/unitree/lab_teleop`.
6. Require normal telemetry from every arm joint and both Dex3 state streams.
7. Test simulation and network-loss handling before authorizing arms-only motion.
8. Keep the R3 remote and a power-stop operator ready for every hardware test.

The stock `xr_teleoperate` real-robot launcher is not passive while it waits for
input: it enters control mode, creates publishers, and starts the arm command
loop. Treat launching it as the beginning of actuation.

## Current Development Paths

- [Meta Quest 3 Teleoperation](../../simulation/unitree/quest-3-teleoperation.md):
  working in Isaac Lab simulation; real-robot control remains gated.
- [GEAR-SONIC](../../simulation/unitree/gear-sonic.md): released whole-body policy
  and local MuJoCo/Vicon experiments; no physical deployment has been approved.
- [Wheelchair Push](../../simulation/unitree/policy-experiments/wheelchair-push/index.md):
  Isaac Lab RL experiments and the articulated wheelchair scene.
- [Vicon LAN](../networking/vicon-lan.md): official Nexus capture path and camera
  network notes.

## Superseded Findings

The March 2026 notes that described all DDS topics as silent and the robot as
unactivatable are historical diagnostics, not the current state. Later sessions
confirmed working DDS state reads and motion-service RPCs. The present blocker
is the left-arm telemetry fault and the need for a controlled hardware preflight.
