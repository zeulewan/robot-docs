# Unitree Explore App

## Overview

The Unitree Explore app is used to control the G1, switch modes, view video, and perform firmware updates.

- **Platform:** iOS/iPadOS only (no Mac version; TestFlight beta is iOS-only)
- **Bluetooth:** Connects via BLE to the locomotion computer's integrated Bluetooth 5.2 (Rockchip RK3588, not the Jetson's)
- **Wi-Fi:** Connects to the locomotion computer's integrated WiFi 6 AP (wlan1, SSID "UnitreeG1", 192.168.12.0/24)
- **Control protocol:** WebRTC via `webrtc_bridge` module on the locomotion computer. BLE is used for initial pairing and WiFi provisioning, then all ongoing control/video uses WiFi + WebRTC.

## Connection Methods

### Bluetooth (stock, working)

The app pairs via BLE to the locomotion computer (RK3588 at 192.168.123.161). The robot doesn't advertise Bluetooth constantly. You need to tap "Add Robot" in the app to trigger pairing mode, then it shows up. Works out of the box with no network setup.

### Built-in Wi-Fi AP (stock, working)

The locomotion computer broadcasts a Wi-Fi AP on **wlan1**:

- **SSID:** UnitreeG1
- **IP:** 192.168.12.1 (separate subnet from the wired 192.168.123.0/24 network)

Connect your iPad/phone to this network for app control. No password appears to be required (or use the default if prompted).

### Wi-Fi Client Provisioning (stock, broken)

The app's Bluetooth provisioning flow tries to connect the locomotion computer's **wlan0** (STA/client mode) to an external Wi-Fi network for internet access. This currently fails, showing "Internet Disconnected" in the Machine Inspection screen.

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
