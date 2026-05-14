# Sunshine Streaming

Sunshine game streaming server setup for on-demand remote desktop access via Moonlight.

Sunshine and the GNOME X11 desktop are normally stopped. Bring them up only when remote desktop access is needed, then shut them back down when finished.

## Service

| | |
|---|---|
| **Type** | systemd user service (`sunshine.service`) |
| **Auto-start** | Disabled |
| **Control script** | `~/bin/stream-desktop` |
| **Start order** | GDM/X11 first, then Sunshine |
| **Override** | `~/.config/systemd/user/sunshine.service.d/override.conf` |
| **Restart while active** | `Restart=always`, `RestartSec=5` |

`sunshine.service` is disabled so it does not automatically start with a graphical session:

```bash
systemctl --user is-enabled sunshine.service
# disabled
```

Use `stream-desktop` instead of starting Sunshine directly. The script starts GDM, waits for the X11 display on `:0`, disables screen blanking, then starts Sunshine. This avoids Sunshine coming up before the NVIDIA X11 desktop is ready, which can make it fall back to VAAPI instead of NvFBC/NVENC.

## On-Demand Desktop

```bash
# Start Moonlight desktop access
stream-desktop start

# Stop Sunshine and the X11 desktop
stream-desktop stop

# Restart the whole stack
stream-desktop restart

# Check current state
stream-desktop status

# Recent Sunshine logs
stream-desktop logs
```

Expected stopped state:

```text
GDM: inactive
Sunshine: inactive
No accessible X11 display on :0
No Sunshine ports listening
```

Expected running state:

- GDM active with user `zeul` autologged into GNOME on X11
- X11 display available on `:0`
- Sunshine active and listening on TCP `47984`, `47989`, `47990`, `48010` plus UDP `47998-48000`
- Web UI available at `https://localhost:47990`

## Config (`~/.config/sunshine/sunshine.conf`)

```ini
nvenc_preset = 1
nvenc_twopass = disabled
fec_percentage = 10
origin_web_ui_allowed = wan
qp = 10
```

## Encoder Path

Healthy startup after `stream-desktop start` should show NvFBC/NVENC in the Sunshine logs:

```text
Screencasting with NvFBC
Found H.264 encoder: h264_nvenc [nvenc]
Found HEVC encoder: hevc_nvenc [nvenc]
```

If logs show `h264_vaapi [vaapi]`, Sunshine likely started before the X11 desktop was fully ready. Run:

```bash
stream-desktop restart
```

## Resolution Auto-Switch

| | |
|---|---|
| **Script** | `~/set-resolution.sh` |
| **Trigger** | Sunshine prep-cmd on stream start |
| **Env vars** | `SUNSHINE_CLIENT_WIDTH` / `SUNSHINE_CLIENT_HEIGHT` |
| **Method** | `nvidia-settings -a CurrentMetaMode` |
| **Log** | `/tmp/sunshine-res.log` |

## Web UI

Available only while Sunshine is running:

`https://localhost:47990` (or via Tailscale IP). Remote access is enabled with `origin_web_ui_allowed = wan`.

## Quick Reference

```bash
# Start/stop/status through the managed wrapper
stream-desktop start
stream-desktop stop
stream-desktop status

# Sunshine logs
stream-desktop logs

# Raw service status
systemctl --user status sunshine

# Confirm autostart remains disabled
systemctl --user is-enabled sunshine
```
