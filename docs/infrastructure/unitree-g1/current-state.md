# Current State

!!! warning "Connectivity is the blocking issue"
    The G1 is physically operational but we cannot reliably connect to it wirelessly. Wired SSH to the Jetson works, but WiFi connectivity (needed for the app, video, and untethered operation) is broken or severely degraded. Everything else is blocked until connectivity is sorted out.

## Goal

Get normal, reliable wireless connectivity to the G1 so that:

1. The Unitree Explore app can connect with full video streaming
2. The Jetson has internet access for SDK installation and development
3. The robot can operate untethered (no ethernet cable to a laptop)

## What Works

| Component | Status | Notes |
|-----------|--------|-------|
| Wired SSH to Jetson | Working | USB ethernet adapter, static IP 192.168.123.100, `ssh unitree@192.168.123.164` |
| Bluetooth pairing | Working | App pairs via BLE to locomotion computer (RK3588) |
| WG827 router admin | Working | SSH/LuCI: `root` / `indr0.com`, WiFi AP broadcasting |
| Internet sharing via pfctl NAT | Documented | Mac forwards traffic from 192.168.123.0/24 to internet via en0 |

## What Doesn't Work

### 1. Built-in WiFi AP (locomotion computer)

The RK3588's wlan1 broadcasts "UnitreeG1" on 192.168.12.0/24. The app connects to this by default (AP mode). However, this puts the app on a **separate subnet** (192.168.12.x) from the wired network (192.168.123.x) with no routing between them. The standard WiFi provisioning flow (where the app sends WiFi credentials via Bluetooth to wlan0) **fails** with "Internet Disconnected."

We don't know if this is a firmware bug, a cloud registration requirement, or something else.

### 2. Video feed (WebRTC)

All three WebRTC services are abnormal on the locomotion computer: `webrtc_bridge`, `webrtc_signal_server`, `webrtc_multicast_responder`. Video streaming is completely broken regardless of connection method. Multiple other services are also down (`ai_sport`, `motion_switcher`, `ros_bridge`, `lidar_driver`).

### 3. Jetson WiFi hotspot (too slow)

We got a workaround running where the Jetson's Realtek wlan0 runs a hostapd AP ("UnitreeG1-Dev") bridged to 192.168.123.0/24. The app connects and basic control works (walking, mode switching). But the throughput is around **120 kbps**, which is far too slow for video. The Jetson's Realtek adapter is 2.4GHz 802.11n only, and the setup doesn't persist across reboots.

## Unexplored: WG827 Router

Indro Robotics shipped a [ZBT WG827 router](hardware.md) velcroed to the robot's back. We got the admin password (`root` / `indr0.com`) but haven't seriously configured it yet. This is likely the intended path to proper connectivity.

### What the WG827 can do

- **WiFi AP on 192.168.123.0/24** - already broadcasting as "UnitreeG1-Router" (password "Temp1234"). Since it's plugged into the G1's internal switch, WiFi clients would be on the same subnet as the Jetson (.164) and locomotion computer (.161). This bypasses the separate-subnet problem entirely.
- **Dual-band WiFi** - 2.4GHz (300 Mbps) + 5GHz (867 Mbps), significantly better than the Jetson's 2.4GHz-only Realtek adapter
- **4G/5G cellular uplink** - has an M.2 slot and SIM card holder for a cellular modem. Could give the robot internet without tethering to a laptop.
- **Gigabit wired backhaul** - connected to the G1's internal switch, so traffic from WiFi clients to the robot's computers goes over Gigabit ethernet internally

### Why we think this is the path forward

The WG827 AP ("UnitreeG1-Router") puts devices on 192.168.123.0/24, the same subnet as the locomotion computer (.161). If the app connects to this WiFi instead of the built-in "UnitreeG1" AP (which is on 192.168.12.0/24), it should be able to reach the robot in STA-L mode via multicast discovery or direct IP. This skips the broken WiFi provisioning entirely and gives much better throughput than the Jetson hotspot.

### What we need to figure out

- Does the app actually work in STA-L mode when connected to the WG827's WiFi?
- Is the WG827's 5GHz band working and does it provide enough throughput for video?
- Can we configure the WG827 to share internet (via Mac NAT or a SIM card) to give the whole robot network internet access?
- What's the WG827's OpenWrt version and can we update its firmware? (It ships with old LEDE 17.01 / ZBT custom firmware, and is not officially supported by mainline OpenWrt)

## Security: UniPwn

While investigating connectivity, we also looked at [UniPwn](https://github.com/Bin4ry/UniPwn), a root-level exploit affecting all Unitree robots (G1, H1, Go2, B2, R1).

- Exploits the BLE WiFi provisioning interface using a **hardcoded AES key** identical across every Unitree robot
- Anyone within Bluetooth range encrypts the word "unitree" with the public key and gets root on the locomotion computer
- The exploit is **wormable** - a compromised robot scans BLE for other Unitree robots and spreads automatically
- Also reveals that configuration files use a static Blowfish-ECB key (same on every G1)

This is relevant because the locomotion computer (RK3588) is currently a black box with no SSH access. UniPwn could potentially be used to gain access for debugging the abnormal services and WiFi issues. It also means the BLE interface should be locked down once connectivity is sorted.

Security research also revealed that the G1 phones home via MQTT telemetry (port 17883) every 300 seconds and streams data to external servers. Worth being aware of if/when we give the robot internet access.

## Prioritized Next Steps

1. **Test the WG827 WiFi with the app** - connect iPad to "UnitreeG1-Router", see if the app discovers the robot in STA-L mode
2. **Give the robot internet** - either via Mac pfctl NAT through the WG827, or explore the 4G/5G SIM slot
3. **Install Unitree SDK2 Python** on the Jetson (requires internet)
4. **Investigate abnormal services** - especially the WebRTC stack, since video won't work until those are fixed regardless of WiFi quality
5. **Consider UniPwn** for gaining shell access to the locomotion computer to debug services directly
