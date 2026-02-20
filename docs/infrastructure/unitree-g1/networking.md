# Networking

## Connecting from Mac

1. Plug USB-A to USB-C ethernet adapter into Mac
2. Connect ethernet cable from Mac to G1 port 4 or 5
3. Set static IP:
```bash
sudo ifconfig en13 192.168.123.100 netmask 255.255.255.0 up
```
4. SSH into the Jetson:
```bash
ssh unitree@192.168.123.164
# password: 123
```

!!! warning
    The Mac's built-in ethernet adapters (en4/5/6) are dead (Thunderbolt virtual ports, show "media: none"). Must use the USB ethernet adapter.

## Network Topology (at school)

```
School Wi-Fi (10.17.x.x)
    |
  Mac en0 (Wi-Fi) ---- internet
  Mac en13 (USB ethernet) ---- 192.168.123.100
    |
  G1 port 4/5
    |
  G1 internal L2 switch
    |
  WG827 Router (192.168.123.1) ---- add-on, WiFi + optional 4G/5G
  Jetson eth0 (192.168.123.164) ---- wlan0 disabled
  Locomotion (192.168.123.161)
    ├── eth0: 192.168.123.161 (wired, internal)
    ├── wlan0: STA mode (client, for internet, disconnected)
    └── wlan1: AP mode (192.168.12.1, SSID "UnitreeG1", built-in hotspot)
          |
        iPad / phone (192.168.12.x) ---- app control via Bluetooth + Wi-Fi
  Livox Mid-360 Lidar (192.168.123.20)
```

!!! note "WG827 is optional"
    The WG827 router velcroed to the robot's back is an add-on by Indro Robotics for WiFi and optional 4G/5G. The G1 has an **internal L2 switch** that connects neck ports 4/5 directly to the Jetson, locomotion computer, and lidar. The WG827 is not required for basic ethernet connectivity.

## Locomotion Computer Wi-Fi (built-in)

The RK3588 locomotion computer has two Wi-Fi interfaces:

- **wlan1 (AP mode):** Broadcasts a hotspot (configurable SSID, likely "UnitreeG1" or "G1-XXXXXX") at 192.168.12.1. This is the default connection for the Unitree Explore app. iPad/phone connects here directly — no router needed.
- **wlan0 (STA/client mode):** For connecting the robot to external Wi-Fi (internet access, OTA updates). The app's Bluetooth provisioning flow sends WiFi credentials to this interface, but it currently fails ("Internet Disconnected"). May require Unitree cloud registration.

!!! note
    The AP network (192.168.12.0/24) and the wired network (192.168.123.0/24) are **separate subnets** with no routing between them by default. The app communicates with the locomotion computer directly on 192.168.12.x in AP mode. In WiFi/STA mode, the app discovers the robot via **multicast using the serial number**.

## Jetson Wi-Fi Hotspot (optional, does not persist after reboot)

The Jetson's Realtek wlan0 is **disabled by default** (radio off, hostapd disabled). It can be re-enabled to provide a bridged WiFi AP that puts iPad/phone on the same 192.168.123.0/24 subnet as the wired network. This is useful for iPad app connectivity when the locomotion computer's built-in AP (192.168.12.0/24) is on a separate, unrouted subnet.

**Important: nmcli/wpa_supplicant AP mode does NOT work with this Realtek adapter** (kicks clients with reason_code=2). Must use hostapd.

### Setup Script

Run these commands on the Jetson via SSH. **None of this persists after reboot.**

```bash
# 1. Tell NetworkManager to ignore wlan0
sudo nmcli device set wlan0 managed no

# 2. Create bridge and move eth0 into it
#    (briefly disrupts SSH - run in one session)
sudo ip link add br0 type bridge
sudo ip link set br0 up
sudo ip link set eth0 master br0
sudo ip addr del 192.168.123.164/24 dev eth0
sudo ip addr add 192.168.123.164/24 dev br0

# 3. Write hostapd config
sudo tee /etc/hostapd/hostapd.conf << EOF
interface=wlan0
bridge=br0
driver=nl80211
ssid=UnitreeG1-Dev
hw_mode=g
channel=6
wmm_enabled=1
macaddr_acl=0
auth_algs=1
wpa=2
wpa_passphrase=Temp1234
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
ieee80211n=1
ht_capab=[SHORT-GI-20]
EOF

# 4. Start hostapd
sudo systemctl unmask hostapd
sudo hostapd -B /etc/hostapd/hostapd.conf

# 5. Allow bridge traffic (Docker's FORWARD policy is DROP)
sudo iptables -I FORWARD -i br0 -o br0 -j ACCEPT

# 6. Start DHCP server for Wi-Fi clients
sudo dnsmasq --interface=br0 --bind-interfaces \
  --dhcp-range=192.168.123.200,192.168.123.250,255.255.255.0,12h \
  --dhcp-option=3,192.168.123.164 \
  --dhcp-option=6,8.8.8.8,8.8.4.4 \
  --log-dhcp --no-daemon &
```

### iPad Connection

- **SSID:** UnitreeG1-Dev
- **Password:** Temp1234
- iPad gets IP in 192.168.123.200-250 range, same L2 subnet as the robot
- Uses a different SSID than the locomotion computer's "UnitreeG1" to avoid confusion

## Sharing Internet with G1

### On Mac (zmac)

```bash
# Enable IP forwarding and NAT (en0 = WiFi with internet)
sudo sysctl -w net.inet.ip.forwarding=1
echo 'nat on en0 from 192.168.123.0/24 to any -> (en0)' | sudo pfctl -ef -
```

### On Jetson

```bash
# Set Mac as default gateway and configure DNS
sudo ip route add default via 192.168.123.100
sudo bash -c 'echo nameserver 8.8.8.8 > /etc/resolv.conf'
```

### On WG827 Router (gives internet to all LAN devices + WiFi clients)

```bash
# SSH into router first
ssh root@192.168.123.1  # password: indr0.com

# Add default route via Mac
ip route add default via 192.168.123.100
echo 'nameserver 8.8.8.8' > /tmp/resolv.conf.auto
```

This gives internet to the WG827 itself and all WiFi clients connected to `UnitreeG1-Router`.

!!! warning
    Do NOT use the WG827's WAN port for internet sharing. OpenWrt blocks SSH on the WAN interface by default, making the router inaccessible. Use the LAN side (neck port) with pfctl NAT instead.

!!! note
    macOS Internet Sharing GUI does not work for this setup. The config plist caches stale interface names. Use pfctl NAT directly instead.

## Known Issues

| Issue | Details |
|-------|---------|
| SSH banner timeout (FIXED) | Jetson sshd hangs 60-90s on reverse DNS lookup. Fixed by adding `UseDNS no` to `/etc/ssh/sshd_config`. Persists across reboots. |
| WG827 router password (RESOLVED) | LuCI web UI at http://192.168.123.1 — Username: `root`, Password: `indr0.com` (zero, not letter O). SSH on port 22 with same credentials. |
| WG827 is optional | The WG827 is an add-on by Indro Robotics. The G1 has an internal switch connecting neck ports to all internal computers. Direct connection to neck port 4/5 works without the WG827. |
| Wi-Fi provisioning | App sends WiFi credentials to RK3588's wlan0 via Bluetooth. Currently fails ("Internet Disconnected"). May require Unitree cloud registration. STA-L (local) mode via WG827 router is an untested workaround. |
| Video feed (WebRTC) | webrtc_bridge, webrtc_signal_server, and webrtc_multicast_responder services are all abnormal on the locomotion computer. Video streaming does not work. |
| Abnormal services | ai_sport, motion_switcher, ros_bridge, lidar_driver, dex3 services all showing abnormal in app. May be related to Jetson or lidar connectivity. |
| Two separate subnets | Wired (192.168.123.0/24) and Wi-Fi AP (192.168.12.0/24) are not routed. App works on 192.168.12.x, development SSH works on 192.168.123.x. |
| HDMI output | Port 9 (USB-C Alt Mode) did not output video even with adapter connected at boot. |
| Docker iptables | FORWARD policy is DROP. Must add `iptables -I FORWARD -i br0 -o br0 -j ACCEPT` if using bridge. |
