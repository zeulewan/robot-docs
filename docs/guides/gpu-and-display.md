# GPU & Display Config

NVIDIA GPU configuration and X11 virtual display setup for headless rendering with Isaac Sim.

## Why X11 (not Wayland)?

- Isaac Sim uses Xlib/OpenGL directly, no Wayland support
- XWayland has issues -- users switch to X11
- Virtual display setup easier on X11
- Sunshine/Moonlight works better with X11 for unattended access

## NVIDIA Persistence Mode

| | |
|---|---|
| **Status** | Enabled (set Feb 2026 to fix 30fps idle issue) |
| **Service** | `nvidia-persistenced` |
| **Override** | `/etc/systemd/system/nvidia-persistenced.service.d/override.conf` |
| **Verify** | `nvidia-smi -q \| grep "Persistence Mode"` -- should show `Enabled` |
| **Manual enable** | `sudo nvidia-smi -pm 1` |

The stock service at `/usr/lib/systemd/system/nvidia-persistenced.service` has `--no-persistence-mode` which defeats the purpose. Override removes that flag.

!!! warning "Persistence Mode Required"

    ### Known Issue: 30fps after idle

    - **Symptom**: Sunshine drops to 30fps after not streaming for a while
    - **Root cause**: Without persistence mode, GPU drops to P8 (low power) when idle, throttling NVENC encoder
    - **Fix applied**:
      1. Enabled persistence mode (`nvidia-smi -pm 1`)
      2. Created systemd override to persist across reboots
      3. Disabled DPMS in xorg.conf and via `xset -dpms`
    - If it recurs: check `nvidia-smi` for P-state (should not be P8) and persistence mode status

---

## X11 / Display Configuration

### xorg.conf

| | |
|---|---|
| **Path** | `/etc/X11/xorg.conf` |
| **Backup** | `/etc/X11/xorg.conf.backup` |
| **Display** | Headless virtual (no physical monitor) |
| **Connected monitor** | `DP-0` (virtual) |
| **DPMS** | Disabled (`Option "DPMS" "false"`) |
| **DPI** | 96x96 |
| **ModeValidation** | Permissive (AllowNonEdidModes, NoVirtualSizeCheck, etc.) |

### Current xorg.conf

```
# Headless virtual display - 2016x1260 (1.5x zoom)

Section "ServerLayout"
    Identifier     "Layout0"
    Screen      0  "Screen0"
    InputDevice    "Keyboard0" "CoreKeyboard"
    InputDevice    "Mouse0" "CorePointer"
EndSection

Section "Files"
EndSection

Section "InputDevice"
    Identifier     "Mouse0"
    Driver         "mouse"
    Option         "Protocol" "auto"
    Option         "Device" "/dev/psaux"
    Option         "Emulate3Buttons" "no"
    Option         "ZAxisMapping" "4 5"
EndSection

Section "InputDevice"
    Identifier     "Keyboard0"
    Driver         "kbd"
EndSection

Section "Monitor"
    Identifier     "Monitor0"
    VendorName     "Virtual"
    ModelName      "Headless"
    Option         "DPMS"
EndSection

Section "Device"
    Identifier     "Device0"
    Driver         "nvidia"
    VendorName     "NVIDIA Corporation"
    BoardName      "NVIDIA GeForce RTX 3090"
    BusID          "PCI:1:0:0"
    Option         "AllowEmptyInitialConfiguration" "True"
    Option         "ConnectedMonitor" "DP-0"
    Option         "ModeValidation" "AllowNonEdidModes,NoVirtualSizeCheck,NoMaxPClkCheck"
EndSection

Section "Screen"
    Identifier     "Screen0"
    Device         "Device0"
    Monitor        "Monitor0"
    DefaultDepth    24
    Option         "AllowEmptyInitialConfiguration" "True"
    SubSection     "Display"
        Depth       24
        Modes      "1920x1080"
    EndSubSection
EndSection
```

### Available Resolutions (MetaModes)

| Resolution | Refresh | Use Case |
|------------|---------|----------|
| 3024x1964 | 60Hz | MacBook Pro 14" native (with notch) |
| 3024x1890 | 60Hz | MacBook Pro 14" native (no notch) |
| 3440x1440 | 60Hz | Ultrawide |
| 3840x2160 | 60Hz | 4K |
| 2560x1440 | 60Hz | 1440p |
| 1920x1080 | 120Hz | 1080p high refresh |
| 1920x1080 | 60Hz | 1080p standard |
| 1600x900 | 60Hz | Low-res fallback |

### Custom Resolution with EDID Generator

Set a custom resolution (e.g., matching MacBook Pro 14" display):

```bash
# Clone EDID generator (if not already done)
cd /tmp && git clone https://github.com/akatrevorjay/edid-generator.git

# Install dependencies
sudo apt-get install -y zsh edid-decode automake dos2unix

# Generate modeline for desired resolution
cvt 2016 1260 60
# Output: Modeline "2016x1260_60.00" 213.00 2016 2152 2368 2720 1260 1263 1269 1307 -hsync +vsync

# Create EDID source file
cd /tmp/edid-generator
echo 'Modeline "2016x1260_60" 213.00 2016 2152 2368 2720 1260 1263 1269 1307 -hsync +vsync ratio=16:10' | ./modeline2edid

# Compile to binary
make 2016x1260_60.bin

# Install EDID
sudo cp 2016x1260_60.bin /etc/X11/edid.bin

# Update xorg.conf with matching modeline, then restart
sudo systemctl restart gdm
```

### Common Resolutions for MacBook Pro 14"

| Resolution | Zoom | Use Case |
|------------|------|----------|
| 3024x1890 | 1x | Native (below notch) |
| 2016x1260 | 1.5x | Comfortable viewing |
| 1512x945 | 2x | Large UI elements |

### DPMS / Screen Blanking

- DPMS disabled (both in xorg.conf and at runtime via `xset -dpms`)
- Screen saver off (`xset s off`)
- Prevents GPU from downclocking when idle

---

## GDM

| | |
|---|---|
| **Autologin** | Enabled (user: zeul) |
| **Wayland** | Disabled (X11 only for NVIDIA KMS capture) |
| **Config** | `/etc/gdm3/custom.conf` |

---

## Troubleshooting

### X server not starting

```bash
# Check logs
journalctl -u gdm -f

# Check Xorg log
tail -50 /var/log/Xorg.0.log

# Restore backup if virtual display broke things
sudo cp /etc/X11/xorg.conf.backup /etc/X11/xorg.conf
sudo systemctl restart gdm
```

### NVIDIA kernel module failed to initialize

If X fails with "Failed to initialize the NVIDIA kernel module":

```bash
# Check if driver is working
nvidia-smi

# If nvidia-smi works but X fails, restart GDM
sudo systemctl restart gdm

# If nvidia-smi fails, check DKMS
dkms status

# Reinstall DKMS module if needed (use installed driver version)
sudo dkms install nvidia-srv/<driver-version>
```

### Rollback xorg.conf

```bash
sudo cp /etc/X11/xorg.conf.backup /etc/X11/xorg.conf
sudo systemctl restart gdm
```

---

## Quick Reference

```bash
# GPU status
nvidia-smi

# Display config
DISPLAY=:0 xrandr

# DPMS status
DISPLAY=:0 xset q | grep -A3 DPMS
```

## File Locations

```
/etc/X11/xorg.conf              # X11 display config
/etc/X11/xorg.conf.backup       # Backup of original config
/etc/X11/edid.bin               # Custom EDID for virtual display
/etc/gdm3/custom.conf           # GDM config (autologin)
/var/log/Xorg.0.log             # X server log
/tmp/edid-generator/            # EDID generator tool (cloned)
```
