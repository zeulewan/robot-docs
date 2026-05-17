# Operator Tools

Small command-line tools used during day-to-day simulation work.

These are convenience wrappers around repeatable workstation operations. They should not contain secrets, keyring passwords, or one-off experiment notes. Put experiment results in the relevant simulation page and keep this page focused on reusable commands.

## Quick Reference

| Tool | Purpose | Normal command |
|---|---|---|
| `stream-desktop` | Start/stop Sunshine + GNOME X11 for Moonlight remote desktop. | `stream-desktop start` / `stream-desktop stop` |
| `isaac-clip` | Render, archive, and send Isaac Lab policy playback videos from project presets. | `isaac-clip send unitree-wheelchair-attached` |
| `latest_video_site.py` | Serve the newest archived Isaac demo video through a fixed browser URL. | `tmux attach -t latest_video_site` |

## Remote Desktop: `stream-desktop`

Use `stream-desktop` when the workstation desktop needs to be viewed through Moonlight. The script starts GDM/X11, waits for `DISPLAY=:0`, disables screen blanking, then starts Sunshine so the stream uses NVIDIA capture/encoding correctly.

```bash
# Start Moonlight desktop access
stream-desktop start

# Stop Sunshine and the X11 desktop
stream-desktop stop

# Restart the whole desktop streaming stack
stream-desktop restart

# Check GDM, X11, display, and Sunshine ports
stream-desktop status

# Recent Sunshine logs
stream-desktop logs
```

Keep Sunshine autostart disabled. The normal workstation state is no desktop stream until `stream-desktop start` is run.

More detail: [Sunshine Streaming](workstation/sunshine.md) and [GPU & Display Config](workstation/gpu-display.md).

## Isaac Playback Videos: `isaac-clip`

`isaac-clip` renders Isaac Lab policy playback videos from named project profiles. It can archive videos locally or send them through a provider. The current providers are:

| Provider | Use |
|---|---|
| `gog` | Send the rendered MP4 by email using the local `gog` CLI. |
| `file` | Render/archive the MP4 and metadata without sending email. |
| `site` | Render/archive, then overwrite the stable MP4/JSON watched by the latest-video page. |

The real CLI is installed as `isaac-clip`. Its source repo is currently `zeulewan/isaac-runclip`; the local checkout is `/home/zeul/GIT/telecli`. Project/view behavior should live in `isaac-clip` config, not in local wrapper scripts.

Project presets live in:

```text
~/.config/isaac-clip/projects.toml
```

The important attached-wheelchair presets are:

```bash
# Current fixed-base relaxed attached standing run
isaac-clip send unitree-wheelchair-fixed-relaxed-stand-attached
isaac-clip send unitree-wheelchair-fixed-relaxed-stand-attached --view two_orbit
isaac-clip views unitree-wheelchair-fixed-relaxed-stand-attached
isaac-clip checkpoints unitree-wheelchair-fixed-relaxed-stand-attached

# Older free-chair relaxed attached standing run
isaac-clip send unitree-wheelchair-relaxed-stand-attached
isaac-clip send unitree-wheelchair-relaxed-stand-attached --view two_orbit
isaac-clip views unitree-wheelchair-relaxed-stand-attached
isaac-clip checkpoints unitree-wheelchair-relaxed-stand-attached

# Older attached-push run
isaac-clip send unitree-wheelchair-attached

# Inspect available views/checkpoints
isaac-clip views unitree-wheelchair-attached
isaac-clip checkpoints unitree-wheelchair-attached

# Print the render command without running it
isaac-clip send unitree-wheelchair-relaxed-stand-attached --dry-run
```

The fixed-base relaxed attached wheelchair views include the diagnostic `play_args = ["--show-wheelchair-urdf-proxy"]`, so those renders show the simplified URDF proxy geometry used for collision/handle inspection.

The current fixed-base relaxed attached wheelchair preset uses `provider = "site"`, `site_dir = "logs/demos/latest-site"`, and `site_name = "latest.mp4"`. That means the normal `isaac-clip send ...` command updates the latest-video page instead of emailing by default:

```text
https://workstation.tailee9084.ts.net:8002/
```

Site delivery can also compress the stable page copy before publishing it. Configure the project send block with:

```toml
compress_site_video = true
site_crf = 30
site_preset = "veryfast"
site_max_width = 1280
```

This keeps the archived render unchanged under `logs/demos/<render>/`, but transcodes `logs/demos/latest-site/latest.mp4` with `ffmpeg` so the browser page loads faster. The site JSON records the original and compressed byte sizes.

To wait for a future checkpoint and update the page automatically:

```bash
isaac-clip watch unitree-wheelchair-fixed-relaxed-stand-attached \
  --target-iteration 12000 \
  --view two_orbit \
  --render
```

To keep updating the page throughout training, use an interval watcher:

```bash
isaac-clip watch unitree-wheelchair-relaxed-push-attached \
  --every-iterations 250 \
  --view two_orbit \
  --render \
  --notify \
  --to <email>
```

That command starts from the next interval after the current latest checkpoint. For example, if the latest checkpoint is `model_12300.pt` and the interval is `250`, the first render target is `model_12500.pt`, then `model_12750.pt`, `model_13000.pt`, and so on. It updates the latest-video page and sends a short email after each successful render.

The notification uses `gog`; keep provider credentials/keyring material out of docs and project config. Omit `--notify` when only the website should update.

The default `unitree-wheelchair-attached` view currently uses:

| Setting | Value |
|---|---|
| Task | `Unitree-G1-29dof-Wheelchair-Dynamic-Push-Attached` |
| Environments | `10` |
| Checkpoint | latest `model_*.pt` from the configured run glob |
| Camera | follow robot, select best-moving robot after `80` steps |
| Orbit | `360 deg` over `500` video steps |
| Output root | `logs/demos` under `unitree_rl_lab` |

The CLI writes both an MP4 and JSON metadata file. The metadata records the project, checkpoint, view settings, and exact render command so videos are reproducible.

The command prints the operational status that would otherwise require side checks: selected checkpoint, run directory, view, output directory, GPU memory/utilization before and after render, detected training process, training-guard decision, render duration, and final output paths.

`isaac-clip` is not a training manager, but it has a render-time training guard. The default `--training-policy auto` checks for a running Isaac Lab `scripts/rsl_rl/train.py` process. If GPU telemetry has enough headroom, training keeps running while the video renders. If free GPU memory or GPU utilization crosses the configured thresholds, the CLI pauses the training process for the render and resumes it afterward.

Useful overrides:

```bash
# Always render alongside training
isaac-clip send unitree-wheelchair-attached --training-policy continue

# Always pause training during the render, then resume it
isaac-clip send unitree-wheelchair-attached --training-policy pause

# Refuse to render if training is already running
isaac-clip send unitree-wheelchair-attached --training-policy fail
```

The pause uses process signals, not checkpointing. It avoids compute contention but does not free training VRAM.

Do not document or commit local email/keyring credentials. If the `gog` provider cannot unlock in a non-interactive shell, unlock the local keyring/session first or run the command from an interactive workstation shell.

## Latest Video Website

A small local dashboard serves the newest archived Isaac demo video at a stable tailnet URL:

```text
https://workstation.tailee9084.ts.net:8002/
```

The server runs from `unitree_rl_lab` in tmux session `latest_video_site`:

```bash
tmux attach -t latest_video_site
```

It is backed by:

```bash
cd /home/zeul/GIT/unitree_rl_lab
./tools/latest_video_site.py --host 127.0.0.1 --port 8002 --title "Latest Isaac Demo Video"
```

For the active wheelchair push run, start it with the render button wired to the current `isaac-clip` project:

```bash
cd /home/zeul/GIT/unitree_rl_lab
./tools/latest_video_site.py \
  --host 127.0.0.1 \
  --port 8002 \
  --title "Latest Isaac Demo Video" \
  --render-project unitree-wheelchair-minimal-x-rail-progress-push-attached \
  --render-view two_orbit \
  --render-training-policy auto
```

Tailscale Serve exposes that local port:

```bash
tailscale serve --https=8002 8002
tailscale serve status
```

The page scans `/home/zeul/GIT/unitree_rl_lab/logs/demos/**/*.mp4` at request time and embeds only the newest MP4. It also shows the video creation time, exposes `/latest.mp4` for direct playback/download, supports byte-range requests for browser seeking, and auto-refreshes when a newer archived render appears.

When `--render-project` is set, the page also shows a **New Video** button. That button runs `isaac-clip send <project> --provider site` on the workstation, using the latest checkpoint selected by the project preset. The status panel shows whether the render is idle/running/succeeded/failed, elapsed time, exit code, and recent `isaac-clip` output. The **Refresh** button reloads the page immediately; the page also polls status and reloads itself after a successful render updates `latest.mp4`.

To update what the page shows, put the desired MP4 under `logs/demos` with a newer timestamp than the previous videos. The current diagnostic wheelchair-URDF proxy clip was archived as:

```text
logs/demos/unitree-wheelchair-urdf-proxy_model_11100_slow_revolve_20260516_170039/model_11100_urdf_proxy_slow_revolve.mp4
```

That clip was rendered with `scripts/rsl_rl/play.py --show-wheelchair-urdf-proxy`, which swaps the playback wheelchair to the diagnostic proxy-visual URDF:

```text
assets/objects/wheelchair/free3d_active_wheelchair/urdf/active_manual_wheelchair_proxy_visual.urdf
```

## When To Use Which View

Use `isaac-clip` for policy playback videos that need to be shared or logged. Use Moonlight through `stream-desktop` when the goal is interactive inspection: Isaac Sim GUI, RViz2, `rtabmap_viz`, or any heavy 3D view that should render locally on the workstation and stream as compressed video.

For Foxglove/Lichtblick over the browser, keep the view lightweight. Avoid heavy accumulated point clouds over the network unless explicitly debugging them.
