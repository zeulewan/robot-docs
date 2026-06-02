# Current State

!!! warning "Robot is in dormant state - cannot be activated"
    The G1 locomotion board (RK3588) boots and runs all DDS services, but publishes zero data on any topic. The robot appears stuck in a boot standby state. WiFi STA mode is broken, SSH is closed, jailbreak is patched on firmware 1.4.5. The app connection is flaky (AP WiFi intermittent). Without activation via app or remote controller, the robot cannot be controlled.

!!! note "TMU field network update"
    A GL.iNet GL-MT3000 field router (`eph107`) now handles TMU WPA2-Enterprise uplink, local 192.168.8.0/24 Wi-Fi, Tailscale, and WebFinder. This is the preferred operator network at school. It does not replace the G1 wired 192.168.123.0/24 DDS network; use it as the Mac's internet/tailnet side while keeping USB Ethernet to the robot for DDS work.

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
| Wired SSH to Jetson | Working | USB ethernet adapter, static IP 192.168.123.100, `ssh unitree@192.168.123.164`, password `123` (note: Jetson IP varies per boot, was .13 previously, now .164) |
| Wired ping to locomotion board | Working | 192.168.123.161, sub-ms latency |
| DDS discovery over wired | Working | 18 DDS participants, 85 publications discovered from locomotion board (March 9 retest) |
| DDS type matching | Working | Confirmed via CycloneDDS trace logs - type hashes match between SDK and firmware |
| DDS reader/writer matching | Working | `reader_add_connection` and `proxy_writer_add_connection` succeed |
| Multicast UDP packets | Working | Raw socket confirms packets arriving from 192.168.123.161 on 239.255.0.1:7401 |
| Bluetooth pairing | Working | App pairs via BLE to locomotion computer (RK3588), MAC: FC:23:CD:91:30:B4 |
| BLE protocol communication | Working | Full handshake, serial number retrieval, WiFi config commands all ACK'd. Serial: E21D6000P9GCAE |
| WG827 router admin | Working | SSH: `root` / `indr0.com`, WiFi SSID: "UnitreeRouter" / "Temp1234" (2.4GHz ch11, WPA2) |
| Internet via Mac NAT | Working / legacy | pfctl NAT on Mac (192.168.123.100) still works for the G1 wired network. At TMU, Mac `en0` should usually be on the GL.iNet field router rather than directly on TMU. |
| GL.iNet field router | Working | `eph107`, LAN 192.168.8.1, Tailscale 100.84.198.19, TMU WPA2-Enterprise uplink, WebFinder enabled |
| WG827 GL.iNet uplink | Working / preferred | Robot-mounted add-on router uses its single 2.4GHz radio as a client of `GL-MT3000-8b4`; local AP disabled. Observed DHCP: 192.168.8.190/24. Jetson reaches internet through WG827 -> GL.iNet -> TMU. |
| WG827 direct TMU uplink | Tested fallback | WPA2-Enterprise works with `wpad` and CA certs after the clock is correct. Prefer GL.iNet upstream so the WG827 does not hold TMU credentials or handle 802.1X. |
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

### 4. SSH Access to Locomotion Board

**No SSH access available.** Port 22 is closed on the locomotion board (192.168.123.161). No known method to enable it on firmware 1.4.5. See `.private/g1-exploit-notes.md` for details on attempts.

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
- App says "connected to access point" but then fails to connect to the robot itself
- "Server inaccessible" errors frequent (catch-22: iPad on robot AP has no internet, but app needs cloud servers)
- Successfully unbound the robot once (March 8), proving the app CAN talk to the robot briefly
- Multiple retry sessions (March 8-9) with dozens of attempts, only sporadic brief connections
- Tried connecting iPad via router WiFi (UnitreeRouter) for internet, then switching to G1 AP - did not help

### 7. Video feed (WebRTC)

All three app-side WebRTC services are abnormal on the locomotion computer, so Unitree Explore video streaming is broken.

Direct Jetson camera streaming works as a workaround. The Jetson runs Unitree's `video_hub_pc4` service on `/dev/video4`, which publishes H.264/RTP multicast on `230.1.1.1:1720` from `eth0`. A short sample decoded successfully at 1280x720, and live playback works from the Mac over the wireless-only GL.iNet -> WG827 route by SSH-bridging that multicast stream.

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

### Current TMU Operator Network

```
TMU Wi-Fi (WPA2-Enterprise)
  |
GL.iNet GL-MT3000 "eph107"
  |-- wwan: TMU DHCP, observed 10.16.144.207/20
  |-- 5 GHz radio: TMU station/uplink only; private 5 GHz AP disabled
  |-- 2.4 GHz AP: GL-MT3000-8b4 local downlink
  |-- LAN: 192.168.8.1/24
  |-- Tailscale: 100.84.198.19
  |
  +-- Mac Wi-Fi (en0) 192.168.8.109
  +-- Jeff Xi Ubuntu host 192.168.8.241 Ethernet
  +-- operator devices / tablets / robot tools 192.168.8.x
```

The GL.iNet gives operator machines stable internet and tailnet access. The Mac still uses a separate USB Ethernet adapter to the G1 neck port for direct SDK/DDS work. `jeffxi-ubuntu` is an Ubuntu 22.04.5 desktop on the GL.iNet LAN; its `eno1` Ethernet has a DHCP reservation at `192.168.8.241`, and Wi-Fi is intentionally disabled.

### Legacy G1 Wired Network

```
Internet
  |
Mac (en0) -- NAT via pfctl (now usually through GL.iNet Wi-Fi)
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

### Current WG827 GL.iNet Uplink Mode

The WG827 is not internal G1 infrastructure; it is an external/add-on router velcroed to the robot and plugged into the robot's 192.168.123.0/24 Ethernet network.

```text
TMU Wi-Fi (WPA2-Enterprise)
  |
GL.iNet GL-MT3000 "eph107" 192.168.8.1
  |-- DHCP reservation: WG827 f8:5e:3c:ee:42:5e -> 192.168.8.190
  |-- static route: 192.168.123.0/24 via 192.168.8.190
  |
WG827 wlan0 192.168.8.190/24
WG827 br-lan 192.168.123.1/24
  |
G1 wired 192.168.123.0/24
  +-- Jetson eth0 192.168.123.164, default via 192.168.123.1
  +-- Locomotion board 192.168.123.161
  +-- Livox 192.168.123.20
```

Verified:

- WG827 uses `GL-MT3000-8b4` as its upstream.
- GL.iNet has a persistent DHCP reservation for WG827 MAC `f8:5e:3c:ee:42:5e` at `192.168.8.190`.
- GL.iNet has a persistent static route for `192.168.123.0/24` via `192.168.8.190`.
- GL.iNet 2.4 GHz backhaul is pinned to channel 11, HE20, with legacy rates disabled.
- WG827 firewall allows GL LAN clients (`192.168.8.0/24`) to reach the robot LAN and WG827 SSH/web/ping.
- DHCP is split: GL.iNet serves `192.168.8.0/24`; WG827 serves only `192.168.123.0/24` and ignores DHCP on `wwan2`.
- Mac Wi-Fi no longer has the old `192.168.123.100` alias; routed access goes through GL.
- Jetson can ping `1.1.1.1` and resolve DNS through the WG827.
- Mac can SSH to the WG827 and Jetson with the GL-to-WG827 Ethernet cable unplugged.
- `UnitreeRouter` AP is disabled in this mode. AP+STA on the same MT7603E radio failed on GoldenOrb; use the GL.iNet AP for operator devices instead.
- Direct TMU from the WG827 was tested and works as a fallback after fixing the router clock, but it is not the preferred mode.

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

## Legacy Mac-NAT Router Config

This section is for the robot-mounted WG827 on the G1 wired network, not the GL.iNet field router.

Use this only when the WG827 is not joined to the GL.iNet or TMU. In the current GL.iNet-uplink mode, the WG827 itself is the robot internet gateway and these Mac NAT commands are not needed.

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

## MITM AP Approach (March 9, tested)

Attempted to create a fake "UnitreeG1_W" WiFi AP on the Jetson to proxy app traffic to the real robot over wired ethernet:

1. Set up hostapd on Jetson wlan0 (2.4GHz, channel 6, WPA2, password "Temp1234")
2. Configured Jetson as 192.168.12.1/24 (same as real robot AP IP)
3. Set up iptables DNAT: all traffic to 192.168.12.1 on wlan0 -> 192.168.123.161 (real robot over wire)
4. MASQUERADE on eth0 for return traffic, FORWARD rules both directions
5. dnsmasq for DHCP on wlan0

**Result:** iPad connected to the fake AP and got internet through the Jetson -> Mac NAT chain. But the Unitree app never sent any traffic to 192.168.12.1. The app only contacted external servers (Unitree cloud, Apple, DNS). The DNAT rule had zero packet matches.

**Conclusion:** The app does not blindly connect to 192.168.12.1. It either uses mDNS/Bonjour discovery or requires cloud server authentication before connecting to the robot locally. The MITM approach needs an mDNS responder or deeper understanding of the app protocol.

## Recommended Next Steps

1. **Contact Unitree support** (highest priority) - provide serial E21D6000P9GCAE, describe: WiFi STA broken, app connectivity flaky, DDS services running but not publishing data. Ask if they can push firmware update or factory reset via ZeroTier while robot has internet through wired NAT setup. Draft Discord message prepared (see below).
2. **Motherboard replacement** - TMU was already considering this. May be the fastest path to a working robot.
3. **Get the app working** - the app is flaky but sometimes connects. If it connects reliably, try pairing the remote controller and activating the robot. Once active, DDS data should flow.
4. **Try the Jetson upgrade web UI** - `http://192.168.123.164/#/` has a "Recover Last Version" button that runs `backup.zip` (231 MB). This only affects the Jetson, not the locomotion board.
5. **Rockchip USB boot mode** (last resort, voids warranty) - open chassis, find RK3588 recovery button, flash firmware via rkdeveloptool. Requires firmware image from Unitree or TheRoboVerse community. No factory reset button exists on the G1.
