# Operator Tools

Small command-line tools used during day-to-day simulation work.

These are convenience wrappers around repeatable workstation operations. They should not contain secrets, keyring passwords, or one-off experiment notes. Put experiment results in the relevant simulation page and keep this page focused on reusable commands.

## Quick Reference

| Tool | Purpose | Normal command |
|---|---|---|
| `stream-desktop` | Start/stop Sunshine + GNOME X11 for Moonlight remote desktop. | `stream-desktop start` / `stream-desktop stop` |
| `isaac-clip` | Render, archive, and send Isaac Lab policy playback videos from project presets. | `isaac-clip send unitree-wheelchair-attached` |

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

The real CLI is installed as `isaac-clip`. Its source repo is currently `zeulewan/isaac-runclip`; the local checkout is `/home/zeul/GIT/telecli`.

There may also be small local wrapper commands in `/home/zeul/bin` for the active experiment. These are convenience presets only; they call `isaac-clip` underneath. The current wheelchair standing wrapper is:

```bash
# Short preview of the active relaxed attached wheelchair run
wvid

# Longer orbit preview of the same project
wvid orbit

# Show the selected run/checkpoint/render command without rendering
wvid --dry-run
```

`wvid` currently expands to the `unitree-wheelchair-relaxed-stand-attached` project and its default `startup_failure` view. Use `isaac-clip` directly when switching projects or inspecting presets.

Project presets live in:

```text
~/.config/isaac-clip/projects.toml
```

The important attached-wheelchair presets are:

```bash
# Current relaxed attached standing run
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

## When To Use Which View

Use `isaac-clip` for policy playback videos that need to be shared or logged. Use Moonlight through `stream-desktop` when the goal is interactive inspection: Isaac Sim GUI, RViz2, `rtabmap_viz`, or any heavy 3D view that should render locally on the workstation and stream as compressed video.

For Foxglove/Lichtblick over the browser, keep the view lightweight. Avoid heavy accumulated point clouds over the network unless explicitly debugging them.
