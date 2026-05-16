# Operator Tools

Small command-line tools used during day-to-day simulation work.

These are convenience wrappers around repeatable workstation operations. They should not contain secrets, keyring passwords, or one-off experiment notes. Put experiment results in the relevant simulation page and keep this page focused on reusable commands.

## Quick Reference

| Tool | Purpose | Normal command |
|---|---|---|
| `stream-desktop` | Start/stop Sunshine + GNOME X11 for Moonlight remote desktop. | `stream-desktop start` / `stream-desktop stop` |
| `isaac-runclip` | Render, archive, and send Isaac Lab policy playback videos from project presets. | `isaac-runclip send unitree-wheelchair-attached` |

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

## Isaac Playback Videos: `isaac-runclip`

`isaac-runclip` renders Isaac Lab policy playback videos from named project profiles. It can archive videos locally or send them through a provider. The current providers are:

| Provider | Use |
|---|---|
| `gog` | Send the rendered MP4 by email using the local `gog` CLI. |
| `file` | Render/archive the MP4 and metadata without sending email. |

The CLI is installed as `isaac-runclip`. Its source repo is `zeulewan/isaac-runclip`; the local checkout is currently `/home/zeul/GIT/telecli`.

Project presets live in:

```text
~/.config/isaac-runclip/projects.toml
```

The important attached-wheelchair preset is:

```bash
# Render the latest checkpoint and email the video
isaac-runclip send unitree-wheelchair-attached

# Same project, shorter view
isaac-runclip send unitree-wheelchair-attached --view short_best

# Inspect available views
isaac-runclip views unitree-wheelchair-attached

# Inspect recent checkpoints
isaac-runclip checkpoints unitree-wheelchair-attached

# Print the render command without running it
isaac-runclip send unitree-wheelchair-attached --dry-run
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

Do not document or commit local email/keyring credentials. If the `gog` provider cannot unlock in a non-interactive shell, unlock the local keyring/session first or run the command from an interactive workstation shell.

## When To Use Which View

Use `isaac-runclip` for policy playback videos that need to be shared or logged. Use Moonlight through `stream-desktop` when the goal is interactive inspection: Isaac Sim GUI, RViz2, `rtabmap_viz`, or any heavy 3D view that should render locally on the workstation and stream as compressed video.

For Foxglove/Lichtblick over the browser, keep the view lightweight. Avoid heavy accumulated point clouds over the network unless explicitly debugging them.
