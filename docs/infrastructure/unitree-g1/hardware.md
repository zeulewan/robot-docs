# Hardware

## Overview

Unitree G1 EDU humanoid robot with Jetson Orin NX development computer.

## Battery and Charging

- **Battery:** 13-string lithium, 46.8V rated, 54.6V charge limit, 9000mAh / 421.2Wh
- **Charger:** 54V / 5A output, 100-240V AC input
- **Charging method:** Remove battery from robot, connect charger directly to battery pack. No DC input on robot body.
- **Charge time:** Approximately 1.5-2 hours
- **Indicator:** LED flashes at 1Hz during charging, turns off when full
- **No tethered/wall power mode.** Robot is battery-only during operation.

## Internal Computers

| Board | IP | Role |
|-------|-----|------|
| Locomotion Computer (RK3588) | 192.168.123.161 (eth0) | Rockchip RK3588 (8-core ARM, 8GB RAM, 32GB eMMC). Runs Unitree locomotion controller, WebRTC bridge, network manager, and all proprietary Unitree services. Has integrated WiFi 6 (wlan0 STA + wlan1 AP at 192.168.12.1, SSID "UnitreeG1") and Bluetooth 5.2 (for app pairing). Not open to user (no SSH). |
| Development Computer (Jetson Orin NX) | 192.168.123.164 | User development PC. Ubuntu 20.04, aarch64. |
| Livox Mid-360 Lidar | 192.168.123.20 | 3D LIDAR for mapping and obstacle detection |
| ZBT WG827 Router | 192.168.123.1 | OpenWrt router added by Indro Robotics (velcroed to back). Provides WiFi (SSID: UnitreeG1-Router) and optional 4G/5G. SSH/LuCI: root / indr0.com |

All boards are connected via an **internal L2 switch** inside the G1 on the 192.168.123.0/24 subnet. Neck ports 4/5 connect directly to this internal switch. The WG827 router is an add-on plugged into the same switch, not the core network bridge.

### Jetson Orin NX Details

- **SSH:** `unitree@192.168.123.164` / password: `123`
- **OS:** Ubuntu 20.04.5 LTS, Linux 5.10.104-tegra, aarch64
- **Wi-Fi:** Realtek wlan0 (disabled, radio off). Supports AP mode via hostapd only (NOT nmcli), but not needed since the locomotion computer has a built-in AP.
- **Bluetooth:** Realtek hci0, named "ubuntu" (separate from the locomotion computer's Bluetooth)
- **Docker:** Installed (sets iptables FORWARD policy to DROP)
- **hostapd:** Installed manually (2:2.9-1ubuntu4.6, via scp). Disabled (not needed).
- **sshd fix:** `UseDNS no` added to `/etc/ssh/sshd_config` (without this, SSH hangs for 60-90s due to reverse DNS lookup on a network with no DNS server)
- **HDMI:** Port 9 (USB-C Alt Mode) did not output video even with adapter connected at boot. May need a specific adapter or Jetson display config.

### ZBT WG827 Router (Shenzhen Zhibotong)

**Model ZBT WG827**, physically velcroed to the robot's back, plugged into the G1's internal switch. Added by Indro Robotics. Made by Shenzhen Zhibotong Electronics.

- **Chipset:** MT7621DA (dual core, 880MHz)
- **WiFi:** 2.4GHz 802.11bgn (MT7603E), 6 SMA antenna ports (2.4GHz)
- **Ports:** 1 WAN + 4 LAN (all Gigabit)
- **Software:** OpenWrt 21.02-SNAPSHOT (GoldenOrb), hostname `InDro_Robotics`
- **M.2 slot + SIM card slot** (for optional 4G/5G module)
- **RAM:** 128MB, Flash: 16MB

The WG827 is **not required** for basic ethernet connectivity. The G1 has an internal L2 switch connecting neck ports (4/5) directly to all internal computers. The WG827 is an add-on for WiFi, plugged into the same switch.

!!! success "Credentials"
    Web UI (LuCI) at `http://192.168.123.1` — Username: `root`, Password: `indr0.com` (zero, not letter O). SSH also open on port 22 with same credentials.

**WiFi AP:**

- **SSID:** `UnitreeG1-Router`, **Password:** `Temp1234`, Channel 11 (2.4GHz), WPA2
- AP (`wlan0`) is bridged to `br-lan` - clients join 192.168.123.0/24, same subnet as Jetson and locomotion computer

!!! warning "wwan2 STA interface"
    The router ships with a secondary STA client (`wwan2`) trying to connect to "Hotspot Manager Interface". This shares the same radio and prevents the AP from coming up (interface stays `NO-CARRIER`). It has been disabled. Do not re-enable it.

## Electrical Interface (back of neck)

| Port | Connector | Description |
|------|-----------|-------------|
| 1 | XT30UPB-F | VBAT - Battery power output (58V/5A). Not a charging input. |
| 2 | XT30UPB-F | 24V/5A power output |
| 3 | XT30UPB-F | 12V/5A power output |
| 4 | RJ45 | GbE - connects to internal L2 switch |
| 5 | RJ45 | GbE - connects to internal L2 switch |
| 6 | Type-C | USB 3.0 host, 5V/1.5A (plug USB devices INTO robot) |
| 7 | Type-C | USB 3.0 host, 5V/1.5A (plug USB devices INTO robot) |
| 8 | Type-C | USB 3.0 host, 5V/1.5A (plug USB devices INTO robot) |
| 9 | Type-C | Alt Mode - USB 3.2 host + DP1.4 (HDMI out via adapter) |
| 10 | 5577 | I/O OUT - 12V/3A power output |

!!! note
    Ports 6-9 are USB **host** ports on the Jetson side. They are for plugging peripherals INTO the robot, not for connecting a laptop.

## Documentation

- [Official Unitree G1 Developer Docs](https://support.unitree.com/home/en/G1_developer)
- Official 32-page manual: `~/Downloads/G1-USER-MANUAL-EN.pdf`
- [Dev guide (Weston Robot)](https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/)
- [Internet guide (Weston Robot)](https://docs.westonrobot.com/tutorial/unitree/g1_internet_guide/)
- [QRE docs (specs, GPIO, electrical)](https://docs.quadruped.de/projects/g1/html/g1_overview.html)
- Downloaded HTML docs: `~/Downloads/G1-docs/`
