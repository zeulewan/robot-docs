# Vicon LAN

Operational notes for the Vicon camera network, the physical Windows Nexus host,
the D-Link PoE switch, `jeff-xi`, and the field-router path.

No credentials, tokens, camera serials, or raw private keys belong in this public
doc. The private copy lives under `/home/zeul/GIT/robot-docs/.secrets/`, which is
ignored by Git.

## Current State

Updated 2026-09-14 from the last verified lab state.

The physical Vicon kit is:

- six Vicon Vero v2.2 motion-capture cameras
- one Vicon Lock Lab sync/interface box
- one D-Link DGS-1520-28MP PoE switch
- one physical Windows Vicon host with Nexus installed
- `jeff-xi`, the Ubuntu operator/capture host
- `eph107`, the GL.iNet field router and Tailscale path
- a native Windows Isaac Lab environment plus WSL 2 for utilities

The cameras are not normal ONVIF, RTSP, or web cameras. They use Vicon-specific
discovery and control. Treating them as generic RGB cameras will not work.

The physical Windows host now has both required subnets on its linked `Ethernet`
adapter: `192.168.8.132/24` for field-router management and `192.168.10.1/24`
for the Vicon camera network. Local SSH and router ping to `192.168.8.132` were
verified on 2026-08-27. Camera and D-Link reachability must still be rechecked
whenever the Vicon kit is recabled or powered up.

The current plan is to use official Vicon Nexus on the physical Windows host. The
old standalone reverse-engineering notes and VM experiments are historical
fallback material, not the primary path.

## Hosts

| Role | Host | Notes |
|---|---|---|
| Field router | `eph107` | GL.iNet GL-MT3000, field LAN gateway `192.168.8.1`, Tailscale reachable |
| Ubuntu operator host | `jeffxi-ubuntu` / `jeff-xi` | Can reach the D-Link management IP when the lab hardware is cabled |
| Physical Vicon Windows host | `vicon-windows` / `OMG-TMU-DAE` | Has Nexus, Sunshine, Tailscale, and OpenSSH |
| Vicon switch | D-Link DGS-1520-28MP | Management IP `10.90.90.90` |

Zeul reaches the physical Windows host through the local field-router path at
`192.168.8.132`. Manraj can use either that local address or the host's separate
Manraj-tailnet identity at `100.84.179.109`. The Vicon host is no longer enrolled
in Zeul's tailnet. Private key paths and account passwords are documented only
in the private secrets file.

## Physical Windows Host

The physical host is distinct from the older `vicon-win11` VM. For current lab
work, assume `vicon-windows` means the physical Vicon/Nexus computer.

Verified on 2026-08-27:

- hostname: `OMG-TMU-DAE`
- user: `Vicon-OEM`
- OpenSSH is running
- Tailscale is running
- Sunshine is running
- Nexus 2.16 is installed
- WSL 2.7.12 with Ubuntu 22.04 is installed
- Sunshine listens on the normal Moonlight ports, including TCP `47984`,
  `47989`, `47990`, and `48010`

Current Windows network state:

| Interface | State |
|---|---|
| `Ethernet` | up at 1 Gbps, `192.168.8.132/24` and `192.168.10.1/24` |
| `Ethernet` field-router reservation | MAC reserved as `192.168.8.132` on `eph107` |
| `ViconMX1` | disconnected |
| `ViconMX2` | disconnected |
| `Wi-Fi` | up on TMU network |
| `Tailscale` | up at `100.84.179.109` on Manraj's separate tailnet |

Reachability from Windows on 2026-06-09:

| Target | Result |
|---|---|
| `192.168.8.241` (`jeff-xi`) | reachable |
| `10.90.90.90` D-Link switch | not reachable from Windows |
| `192.168.10.10-.16` Vicon camera Telnet/control ports | not reachable from Windows |

The 2026-06-09 failures pointed to a cabling, VLAN, or duplicate-host-IP issue,
not a Nexus install issue. On 2026-08-27, `192.168.8.132` had been removed from
Windows while `192.168.10.1` remained. Restoring `.132` as a persistent secondary
address fixed router visibility without changing the Vicon camera address or
adding another default gateway. Windows Firewall permits ICMP echo only from
`192.168.8.0/24`, and local SSH remains available on TCP 22.

## Windows Isaac Lab and WSL

Isaac Sim and Isaac Lab are installed **natively on Windows** for Manraj. The
verified stack includes Isaac Sim `5.1.0.0`, Isaac Lab `2.3.2`, Python `3.11.16`,
PyTorch `2.7`, and Unitree RL Lab in the `env_isaaclab` Conda environment. A
four-environment, one-iteration G1 velocity-policy smoke test completed on the
RTX 4060 and wrote a checkpoint.

See [Isaac Lab on the Vicon Windows Host](../../simulation/unitree/windows-isaac-lab.md)
for the setup and local handoff-file paths.

WSL is retained for Linux command-line tools and ROS-side work. It is not the
Isaac Sim runtime because its Vulkan path was not usable for Kit/PhysX rendering.

WSL was installed and verified on 2026-08-27 without changing the Windows
NVIDIA driver:

| Component | Verified state |
|---|---|
| WSL | `2.7.12.0`, default version 2 |
| Kernel | `6.18.33.2-2-microsoft-standard-WSL2` |
| Distribution | Ubuntu `22.04.5 LTS` |
| Default Linux user | `manraj`, member of `sudo` |
| Python | `3.10.12` |
| Git | `2.34.1` |
| GCC | `11.4.0` |
| GPU | RTX 4060 visible through `/dev/dxg` |
| WSL CUDA view | driver `560.94`, CUDA `12.6`, 8188 MiB VRAM |

The baseline packages are `git`, `curl`, `build-essential`, `python3-venv`,
`python3-pip`, and `openssh-client`. The package database passed `dpkg --audit`
after installation.

Enter the default distribution from a Windows shell:

```powershell
wsl -d Ubuntu-22.04
```

Run a non-interactive Linux command through Windows SSH:

```powershell
wsl -d Ubuntu-22.04 -- bash -lc "whoami; nvidia-smi"
```

Do not install a Linux NVIDIA display driver inside WSL. GPU access is provided
by the Windows host driver through `/usr/lib/wsl/lib`.

## Vicon Network Shape

Expected control subnet:

```text
Vicon host: 192.168.10.1/24
cameras:    192.168.10.10-.16
```

Observed protocol behavior:

```text
UDP 8568 -> 8567    Vicon DHCP-like discovery
UDP 8570            camera heartbeat to host
Telnet 23/tcp       opens on cameras after correct Vicon IP assignment
UDP 4000/6000       centroid/greyscale stream family
UDP 7000/8000       video stream family
```

The host at `192.168.10.1` must be unique. If Windows marks that address
`Duplicate`, Nexus will not be able to own the Vicon host role cleanly.

## D-Link Switch

Observed switch facts:

- model: D-Link DGS-1520-28MP
- management IP: `10.90.90.90`
- HTTP management available when on the correct layer-2 path
- Telnet management available when on the correct layer-2 path
- VLAN state was previously flat VLAN 1 with all relevant ports untagged
- PoE camera ports were previously `eth1/0/2-.7`
- Lock Lab was previously on `eth1/0/24`

PoE restore gotcha:

```text
poe power-inline never   disables PoE
no poe power-inline      restored default/auto delivery during testing
poe power-inline auto    did not reliably restore delivery
```

## Recommended Bring-Up

1. Cable the Vicon switch, cameras, Lock Lab, `jeff-xi`, and the physical Windows
   host onto the intended lab layer-2 network.
2. Confirm `jeff-xi` can reach the switch at `10.90.90.90`.
3. Confirm exactly one machine owns `192.168.10.1/24` on the Vicon layer-2
   network.
4. If Windows/Nexus should drive the cameras, remove any competing
   `192.168.10.1` address from `jeff-xi` before launching Nexus.
5. Confirm the physical Windows host can reach:

```text
10.90.90.90
192.168.10.10:23
192.168.10.11:23
192.168.10.12:23
192.168.10.13:23
192.168.10.14:23
192.168.10.15:23
192.168.10.16:23
```

6. Launch Nexus on the physical Windows host.
7. Capture the official startup/control traffic from `jeff-xi`:

```bash
sudo tcpdump -i br-vicon-lab -w /tmp/vicon_official_handshake.pcap -s 0 \
  'net 192.168.10.0/24 or udp port 8567 or udp port 8568 or udp port 8570'
```

8. If needed, compare the Nexus startup packets against the historical manual
   Telnet and DHCP-like discovery attempts. Keep Nexus/official software as the
   primary path.

## Private Material

Secrets and local-only details are stored inside the main repo working tree but
outside Git:

```text
/home/zeul/GIT/robot-docs/.secrets/vicon-lan-secrets.md
/home/zeul/GIT/robot-docs/.secrets/robot-infrastructure-secrets.md
/home/zeul/GIT/robot-docs/.secrets/vicon_fixed_leases.zsh
```

On zmac, related copies and key files are under:

```text
/Users/zeul/.secrets/
/Users/zeul/.ssh/
```

Do not copy passwords, API tokens, switch credentials, TMU credentials, camera
serial/MAC mappings, or private key blocks into `robot-docs`.
