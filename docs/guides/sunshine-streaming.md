# Sunshine Streaming

Sunshine game streaming server setup for remote desktop access via Moonlight.

## Service

| | |
|---|---|
| **Type** | systemd user service (`sunshine.service`) |
| **Auto-start** | On graphical session |
| **Override** | `~/.config/systemd/user/sunshine.service.d/override.conf` |
| **Restart** | `Restart=always`, `RestartSec=5` |

## Config (`~/.config/sunshine/sunshine.conf`)

```ini
nvenc_preset = 1
nvenc_twopass = disabled
fec_percentage = 10
origin_web_ui_allowed = wan
qp = 10
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

`https://localhost:47990` (or via Tailscale IP). Remote access enabled (`origin_web_ui_allowed = wan`).

## Quick Reference

```bash
# Sunshine logs
journalctl --user -u sunshine -n 50

# Restart Sunshine
systemctl --user restart sunshine

# Service status
systemctl --user status sunshine
```
