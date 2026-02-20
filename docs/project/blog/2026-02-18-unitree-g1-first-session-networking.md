# Unitree G1 First Session — Hardware, Networking, and App Setup

*2026-02-18*

First hands-on session with the Unitree G1 EDU. Got ethernet connectivity working, set up the Jetson as a WiFi bridge, fixed SSH, figured out the WG827 router, and got the app connecting. A lot of things that looked straightforward turned out to have quirks.

## Ethernet from Mac to G1

The G1 exposes an internal L2 switch via neck ports 4 and 5. Plugging directly into either port puts you on the 192.168.123.0/24 subnet with the Jetson (164), locomotion computer (161), and lidar (20).

The Mac's built-in ethernet adapters (en4/5/6 — Thunderbolt virtual ports) all show "media: none" and don't actually work. Had to use a USB-A to USB-C ethernet adapter (comes up as en13) and set a static IP:

```bash
sudo ifconfig en13 192.168.123.100 netmask 255.255.255.0 up
ssh unitree@192.168.123.164  # password: 123
```

## SSH Banner Timeout Fix

SSHing into the Jetson was hanging for 60-90 seconds before showing the login prompt. Root cause: the Jetson's sshd was doing a reverse DNS lookup on the connecting IP, and there's no DNS server on the 192.168.123.0/24 subnet, so it was timing out every connection.

Fix — added one line to `/etc/ssh/sshd_config` on the Jetson:

```
UseDNS no
```

SSH now connects instantly. This persists across reboots.

## WG827 Router

Indro Robotics velcroed a ZBT WG827 router to the robot's back and plugged it into the internal switch. It's an add-on for WiFi and optional 4G/5G, not required for basic ethernet connectivity. The LuCI web UI is at `http://192.168.123.1`.

The password wasn't in any docs. Emailed Indro Robotics and they responded within a couple hours: **root / indr0.com** (that's a zero, not the letter O). SSH on port 22 uses the same credentials.

The WG827 is optional — the G1's internal switch connects neck ports directly to the Jetson, locomotion computer, and lidar regardless of whether the router is powered on.

## Jetson WiFi Hotspot (hostapd)

To get the iPad app working on the same subnet as the Jetson (192.168.123.0/24 instead of the locomotion computer's separate 192.168.12.0/24 AP), I set up the Jetson's Realtek wlan0 as a bridged WiFi AP.

**nmcli and wpa_supplicant don't work** with this Realtek adapter for AP mode — they associate clients but immediately kick them with `reason_code=2`. Must use hostapd directly.

The setup bridges wlan0 into br0 with eth0, so WiFi clients land on the same L2 segment as the wired network:

```bash
sudo nmcli device set wlan0 managed no
sudo ip link add br0 type bridge
sudo ip link set br0 up
sudo ip link set eth0 master br0
sudo ip addr del 192.168.123.164/24 dev eth0
sudo ip addr add 192.168.123.164/24 dev br0
sudo hostapd -B /etc/hostapd/hostapd.conf
sudo iptables -I FORWARD -i br0 -o br0 -j ACCEPT
sudo dnsmasq --interface=br0 --bind-interfaces \
  --dhcp-range=192.168.123.200,192.168.123.250,255.255.255.0,12h ...
```

**SSID:** UnitreeG1-Dev / **Password:** Temp1234. None of this persists after reboot.

The Docker daemon sets `iptables FORWARD policy DROP` on startup, which blocks bridge traffic — the `iptables -I FORWARD` line fixes that.

## Internet Sharing from Mac

To give the G1 (and iPad) internet access through the Mac's school WiFi:

```bash
# Mac
sudo sysctl -w net.inet.ip.forwarding=1
echo 'nat on en0 from 192.168.123.0/24 to any -> (en0)' | sudo pfctl -ef -

# Jetson
sudo ip route add default via 192.168.123.100
sudo bash -c 'echo nameserver 8.8.8.8 > /etc/resolv.conf'
sudo sysctl -w net.ipv4.ip_forward=1
```

macOS Internet Sharing GUI does not work for this — the config plist caches stale interface names. Use pfctl NAT directly.

## App Connectivity

The Unitree Explore app (iOS/iPadOS only) connects via two paths:

- **Bluetooth:** Pairs to the locomotion computer's integrated BT 5.2. The robot doesn't advertise constantly — you need to tap "Add Robot" in the app first to trigger pairing mode. Works out of the box.
- **WiFi:** Connects to the locomotion computer's "UnitreeG1" AP (192.168.12.0/24). This is a separate subnet from the wired network — no routing between them by default.

Mode switching and joystick control worked when connected via the Jetson hotspot (putting the iPad on 192.168.123.0/24). Untested via the built-in "UnitreeG1" AP.

## Known Issues After Session

- **WebRTC broken:** All three video streaming services (webrtc_bridge, webrtc_signal_server, webrtc_multicast_responder) show abnormal in the app. No video feed.
- **Many abnormal services:** ai_sport, motion_switcher, ros_bridge, lidar_driver, dex3, audio_player all abnormal. May be related to Jetson or lidar not being fully connected/configured.
- **WiFi provisioning broken:** App's Bluetooth flow to connect wlan0 (STA mode) to external WiFi fails with "Internet Disconnected".
- **Jetson hotspot doesn't persist:** All the bridge/hostapd/dnsmasq setup is gone after reboot.
