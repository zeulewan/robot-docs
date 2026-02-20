# Unitree Explore App

## Overview

The Unitree Explore app is used to control the G1, switch modes, view video, and perform firmware updates.

- **Platform:** iOS/iPadOS only (no Mac version; TestFlight beta is iOS-only)
- **Bluetooth:** Connects via BLE to the locomotion computer's integrated Bluetooth 5.2 (Rockchip RK3588, not the Jetson's)
- **Wi-Fi:** Connects to the locomotion computer's integrated WiFi 6 AP (wlan1, SSID "UnitreeG1", 192.168.12.0/24)
- **Control protocol:** WebRTC via `webrtc_bridge` module on the locomotion computer. BLE is used for initial pairing and WiFi provisioning, then all ongoing control/video uses WiFi + WebRTC.

## Connection Modes

The app supports three connection modes. The mode is selected during initial binding via the Unitree Explore app.

### AP Mode — Direct Connection (default, working)

The RK3588 locomotion computer broadcasts a Wi-Fi hotspot on **wlan1**:

- **SSID:** Configurable during initial binding (likely defaults to "UnitreeG1" or "G1-XXXXXX")
- **Subnet:** 192.168.12.0/24, gateway 192.168.12.1
- **No router or internet required**

iPad/phone connects directly to this hotspot. The app communicates with the locomotion computer at 192.168.12.1 via WebRTC. This is the default out-of-box experience.

### WiFi/STA Mode — Shared Network (provisioning broken)

In this mode, the app uses **Bluetooth** to send WiFi credentials (SSID + password) to the RK3588, which then joins that external WiFi network as a client on **wlan0** (STA mode). The iPad also connects to the same WiFi network, and they communicate over the shared LAN.

The app discovers the robot via **multicast using the robot's serial number** (not mDNS). This mode may also require Unitree cloud registration — some Go2 users report WiFi mode gets stuck at "register to cloud."

**Currently broken** on our G1 — the Bluetooth provisioning flow fails, showing "Internet Disconnected" in Machine Inspection. Unknown whether this is a firmware bug or a cloud registration issue.

#### STA Sub-modes

The WebRTC connection protocol supports two STA variants:

- **STA-L (Local):** Robot and iPad on the same LAN. Discovery via multicast by serial number or direct IP.
- **STA-T (Tunnel):** Remote connection through Unitree's TURN server using cloud account credentials.

### WG827 Router Workaround (untested)

Since the RK3588 is already on 192.168.123.0/24 via its wired eth0, connecting the iPad to the WG827 router's WiFi (same subnet) should allow the app to reach the robot in STA-L mode — bypassing both the broken WiFi provisioning and the slow Jetson hotspot. Full Gigabit wired backhaul to the RK3588.

### Jetson WiFi Hotspot Workaround (working, slow)

We set up a hostapd AP on the Jetson ("UnitreeG1-Dev") bridged to 192.168.123.0/24. The iPad connects to this and can reach the locomotion computer at .161 on the same subnet. Basic control (walking, mode switching) works, but bandwidth is too low for video — the Jetson's Realtek WiFi is 2.4GHz 802.11n only. See [Networking](networking.md#jetson-wi-fi-hotspot-optional-does-not-persist-after-reboot) for setup.

### Bluetooth (pairing only)

BLE connects to the RK3588's integrated Bluetooth 5.2. The robot doesn't advertise constantly — tap "Add Robot" in the app to trigger pairing mode. Bluetooth is used for initial device binding and WiFi provisioning, not for ongoing control. All ongoing control/video uses WiFi + WebRTC.

## Service Status

From the app's "Automatic Machine Inspection" and "Service Status" screens:

### Functional Services

bashrunner, basic_service, battery_guard, log_system, master_service, net_switcher, robot_state, robot_type_service, state_estimator, unitree_slam, vui_service

### Abnormal Services

| Service | Notes |
|---------|-------|
| ai_sport / 8.4.2.222 | Main locomotion controller |
| motion_switcher / 1.0.0.1 | Mode switching |
| webrtc_bridge / 1.0.8.4 | Video streaming |
| webrtc_signal_server | Video streaming |
| webrtc_multicast_responder | Video streaming |
| ros_bridge / 1.0.0.4 | ROS communication (may need Jetson) |
| lidar_driver / 1.0.0.5 | Livox Mid-360 lidar |
| dex3_service_l / dex3_service_r | Dexterous hand services (may not be installed) |
| audio_player_service | Audio |
| chat_go / 1.9.1.62 | Voice/chat |
| g1_arm_example / 2.0.0.11 | Arm demo |

!!! warning "Video feed broken"
    All three WebRTC services (bridge, signal_server, multicast_responder) are abnormal. This is why video streaming does not work, regardless of Wi-Fi speed.

## App Features

| Feature | Status |
|---------|--------|
| Bluetooth pairing | Working |
| Mode switching (damping, free motor) | Previously working via Jetson hotspot, untested via built-in AP |
| Joystick control | Previously working via Jetson hotspot, untested via built-in AP |
| Video feed | Not working (WebRTC services abnormal) |
| Firmware updates (OTA) | Needs internet on robot |
| Machine Inspection | Working (shows network, USB, Bluetooth status) |
| Service Status | Working (shows all service health) |
| Wi-Fi provisioning | Broken (fails to configure wlan0 client connection) |
