# Vicon LAN Access

Notes for the Vicon/D-Link lab network and the Windows Vicon host.

## Quick Access

### Windows Vicon Host

| | |
|---|---|
| **LAN IP** | `192.168.8.132` |
| **Hostname** | `OMG-TMU-DAE` |
| **SSH user** | `Vicon-OEM` |
| **SSH key on zmac** | `~/.ssh/vicon_oem_ed25519` |

Connect from zmac:

```sh
ssh -i ~/.ssh/vicon_oem_ed25519 Vicon-OEM@192.168.8.132
```

Confirmed on 2026-06-05:

- SSH works after the Windows user signs out.
- `sshd` is running and set to automatic startup.
- The SSH session is elevated admin:
  - User: `omg-tmu-dae\vicon-oem`
  - Group includes `BUILTIN\Administrators`
  - Integrity level is `Mandatory Label\High Mandatory Level`
- Sleep and hibernate are disabled for both AC and battery.
- Display sleep is disabled on AC.
- Active power scheme observed: `Vicon`.

### OpenSSH Fix Applied

Windows OpenSSH initially rejected the Mac key. The working fix was to force the user authorized-keys path and relax strict mode:

```text
PubkeyAuthentication yes
AuthorizedKeysFile .ssh/authorized_keys
StrictModes no
```

Key file:

```text
C:\Users\Vicon-OEM\.ssh\authorized_keys
```

Config backup from the working fix:

```text
C:\ProgramData\ssh\sshd_config.backup-20260605-153245
```

Useful setup/recovery scripts on zmac:

```text
/Users/zeul/Desktop/enable-ssh.ps1
/Users/zeul/Desktop/reset-sshd-key-config.ps1
/Users/zeul/Desktop/diagnose-sshd-auth.ps1
```

## Vicon Network Shape

The Vicon devices expect a Vicon-style host on `192.168.10.1/24`.

Observed protocol shape:

- Vicon DHCP-like discovery: UDP `8568 -> 8567`
- Host should assign devices in `192.168.10.0/24`
- Devices send heartbeat/registration packets to host UDP `8570`
- Device Telnet CLI opens after they are on the expected Vicon subnet
- Camera stream output appears gated by Vicon control/sync commands; direct IP/Telnet access works, but raw frame streaming was not fully unlocked

Helper scripts on zmac:

```sh
cd /Users/zeul
./vicon_fixed_dhcp.sh
```

This starts a Vicon-style DHCP responder for one hour and drains UDP `8570` so macOS does not reply with port-unreachable packets.

## D-Link Switch

| | |
|---|---|
| **Model** | `DGS-1520-28MP` |
| **Hardware** | `A1` |
| **Firmware/runtime seen** | `1.00.029` |
| **Role** | Flat Layer-2 switch for Vicon and lab LAN |

Available management services after setup:

- `22/tcp` SSH
- `23/tcp` Telnet
- `80/tcp` HTTP
- `443/tcp` filtered

Switch behavior:

- Only VLAN 1 was present.
- All ports were untagged VLAN 1.
- No routed/VLAN segmentation was configured.

PoE gotcha:

- `poe power-inline never` disables PoE on a port.
- `poe power-inline auto` did not reliably restore delivery on this switch.
- `no poe power-inline` restored default/auto delivery correctly.

## Vicon Device Layout

Operational fixed layout:

```text
192.168.10.10-.12  Vero v2.2 cameras
192.168.10.13      Vicon Lock Lab
192.168.10.14-.16  Vero v2.2 cameras
```

Exact fixed lease map, MAC addresses, device IDs, and serial numbers are kept out of public docs.

## Camera Details

The six Vero devices reported this common profile via Telnet `info`:

```text
Display Device Type: Vero v2.2
Device Type: VeroV22
Camera Type: Smartcam
Sensor: Vero 2.2
Sensor Dimensions: 2048 x 1088
Ethernet Code Version: 1.0.0+a5da955a built Jul 15 2024 16:45:20
FPGA Firmware Version: 00c4ab23 built Jul 03 2024
Nios2 SDK Version: 22.1std
Firmware Target: Signet
Firmware Version: 810
Bundle: 810
Frame Rate Range: 23.80 - 331
Max Frame Rate Non Windowed: 331
Stream: V2 7000 8000 Video
Stream: V2 6000 4000 Centroids Greyscale
Allowable Gains: 1 2 4 8
Video Gain Range: 0 - 16.0
Video Exposure Range: 0 - 1000
Strobe Type: IR
Strobe Range: 0 - 1000
Ethernet Master Able: yes
Data Redirection Able: yes
Video Able: yes
Accelerometer Able: yes
```

Useful read-only-ish commands:

```text
info
version
help
?
getframerate
getthreshold
getexposure
exptime
dims
window
windowsize
videomode
strobeinfo
greymodeshow
lockshow
```

Useful control knobs discovered:

```text
dest-interface a.b.c.d AA:BB:CC:DD:EE
datadisable [true|false]
coordmode {some|none|only}
greymode {some|none|all|only}
frate [-m][-s][-v][-c clk] [rate]
syncmode freerun
exposure value
gain val
setthreshold val
threshold [-p] [value]
strobeenable 1|0
setstrobe value
videomode [auto|1080|720]
pktsize [-u size][-v size][size]
sendmaskeddata on | off
```

## What Worked

Add the Mac alias for Vicon host emulation:

```sh
sudo ifconfig en0 alias 192.168.10.1 netmask 255.255.255.0 broadcast 192.168.10.255
```

Run the Vicon DHCP helper:

```sh
cd /Users/zeul
./vicon_fixed_dhcp.sh
```

Verify heartbeats:

```sh
sudo tcpdump -i en0 -n -e 'udp port 8570 and net 192.168.10.0/24'
```

Telnet after clean Vicon DHCP:

```sh
telnet 192.168.10.10 23
```

Then:

```text
info
help
```

## What Did Not Work

- Treating the cameras as normal LAN cameras.
- HTTP/RTSP/ONVIF probing.
- Normal DHCP on `67/68`.
- Leaving them on the normal router LAN; they heartbeat but Telnet/control behavior is wrong or incomplete.
- Manual `dest-interface` plus `datadisable false` alone; no stream came out.
- `poe power-inline auto` as the restore command on the D-Link; use `no poe power-inline`.

## Next Step

The fastest route to raw data is to run official Vicon Nexus/Tracker/Shogun/Evoke once on a host at `192.168.10.1` and capture the startup/control traffic:

```sh
sudo tcpdump -i en0 -w /tmp/vicon_official_handshake.pcap -s 0 \
  'net 192.168.10.0/24 or udp port 8567 or udp port 8568 or udp port 8570'
```

Compare official host broadcast/control packets against manual Telnet attempts, then clone the minimum commands needed to start `V2 6000/4000` centroid/greyscale or `V2 7000/8000` video streams.
