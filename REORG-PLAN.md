# Documentation Reorganization Plan

## Current State

15 markdown files across 5 sections (Overview, Isaac Sim, Isaac ROS, Workstation, Networking).
Key issues:
- "Overview" section is a catch-all (getting started, architecture, setup snapshot, progress tracker, about page)
- Significant content overlap between files (gpu-config/x11-display, architecture/isaac-ros, topology/hardware)
- No clear separation between tutorials, reference, and how-to guides
- Planning/status docs mixed with permanent documentation
- Missing docs: no Moonlight setup guide, no G1 robot config, no step-by-step tutorials

## Proposed Structure (Diátaxis Framework)

Reorganize into four clear categories following the [Diátaxis framework](https://diataxis.fr/):

```
docs/
  index.md                          # Landing page (keep)

  getting-started/
    index.md                        # Quick start guide (from getting-started.md, streamlined)
    architecture.md                 # System architecture overview (from architecture.md)

  guides/                           # How-to guides (task-oriented)
    isaac-sim-setup.md              # Install & run Isaac Sim (from isaac-sim/index.md)
    isaac-ros-container.md          # Isaac ROS container usage (from isaac-ros/index.md)
    gpu-and-display.md              # GPU config + X11 display (merge gpu-config.md + x11-display.md)
    sunshine-streaming.md           # Sunshine/Moonlight setup (extract from gpu-config.md + new content)
    tailscale-setup.md              # Tailscale config & ACLs (from networking/tailscale.md)
    nat-troubleshooting.md          # NAT debugging (from networking/nat.md)

  reference/                        # Reference material (information-oriented)
    hardware.md                     # Hardware specs & access (from workstation/hardware.md)
    network-topology.md             # Network layout & IPs (from networking/topology.md)
    current-setup.md                # Software versions snapshot (from current-setup.md)
    isaac-ros-dockerfile.md         # Dockerfile plan/spec (from isaac-ros/dockerfile-plan.md)

  project/                          # Project meta
    progress.md                     # Current progress & next steps (from current-progress.md)
    about.md                        # About this site (from about.md)
```

## Changes Summary

### Merges
- **gpu-config.md + x11-display.md** → `guides/gpu-and-display.md` (heavy overlap on xorg.conf, display config)

### Splits
- **gpu-config.md** → extract Sunshine/Moonlight content into `guides/sunshine-streaming.md`

### Renames/Moves
| Current | New |
|---------|-----|
| getting-started.md | getting-started/index.md |
| architecture.md | getting-started/architecture.md |
| isaac-sim/index.md | guides/isaac-sim-setup.md |
| isaac-ros/index.md | guides/isaac-ros-container.md |
| isaac-ros/dockerfile-plan.md | reference/isaac-ros-dockerfile.md |
| workstation/hardware.md | reference/hardware.md |
| workstation/gpu-config.md | guides/gpu-and-display.md (merged) |
| workstation/x11-display.md | (merged into gpu-and-display.md) |
| networking/topology.md | reference/network-topology.md |
| networking/tailscale.md | guides/tailscale-setup.md |
| networking/nat.md | guides/nat-troubleshooting.md |
| current-setup.md | reference/current-setup.md |
| current-progress.md | project/progress.md |
| about.md | project/about.md |

### Updated Navigation (zensical.toml)

```toml
nav = [
  { "Home" = "index.md" },
  { "Getting Started" = [
    "getting-started/index.md",
    "getting-started/architecture.md",
  ]},
  { "Guides" = [
    "guides/isaac-sim-setup.md",
    "guides/isaac-ros-container.md",
    "guides/gpu-and-display.md",
    "guides/sunshine-streaming.md",
    "guides/tailscale-setup.md",
    "guides/nat-troubleshooting.md",
  ]},
  { "Reference" = [
    "reference/hardware.md",
    "reference/network-topology.md",
    "reference/current-setup.md",
    "reference/isaac-ros-dockerfile.md",
  ]},
  { "Project" = [
    "project/progress.md",
    "project/about.md",
  ]},
]
```

### Updated Sidebar (Astro astro.config.mjs)

```javascript
sidebar: [
  { label: 'Getting Started', items: [
    { label: 'Quick Start', slug: 'getting-started' },
    { label: 'Architecture', slug: 'getting-started/architecture' },
  ]},
  { label: 'Guides', items: [
    { label: 'Isaac Sim Setup', slug: 'guides/isaac-sim-setup' },
    { label: 'Isaac ROS Container', slug: 'guides/isaac-ros-container' },
    { label: 'GPU & Display Config', slug: 'guides/gpu-and-display' },
    { label: 'Sunshine Streaming', slug: 'guides/sunshine-streaming' },
    { label: 'Tailscale Setup', slug: 'guides/tailscale-setup' },
    { label: 'NAT Troubleshooting', slug: 'guides/nat-troubleshooting' },
  ]},
  { label: 'Reference', items: [
    { label: 'Hardware & Specs', slug: 'reference/hardware' },
    { label: 'Network Topology', slug: 'reference/network-topology' },
    { label: 'Current Setup', slug: 'reference/current-setup' },
    { label: 'Isaac ROS Dockerfile', slug: 'reference/isaac-ros-dockerfile' },
  ]},
  { label: 'Project', items: [
    { label: 'Progress', slug: 'project/progress' },
    { label: 'About', slug: 'project/about' },
  ]},
],
```

## AI Readability

### 1. llms.txt Standard
- Add `/llms.txt` and `/llms-full.txt` files to the site root
- Machine-readable site map for AI assistants (LLMs, Claude, etc.)
- Simple markdown file listing all pages with descriptions
- Helps Claude Code and other AI tools understand and reference your docs
- Spec: https://llmstxt.org/

### 2. MCP Server (Future)
- Expose documentation as an MCP tool so Claude Code can query it directly
- Would let you ask Claude about your own docs without web fetches
- Consider once docs stabilize

## Execution Steps

1. Create new directory structure (`getting-started/`, `guides/`, `reference/`, `project/`)
2. Move/rename files per the table above
3. Merge gpu-config.md + x11-display.md into guides/gpu-and-display.md
4. Extract Sunshine content into guides/sunshine-streaming.md
5. Update all internal cross-references between docs
6. Update zensical.toml nav
7. Update astro.config.mjs sidebar
8. Add llms.txt to both sites
9. Test both dev servers locally
10. Commit and push both repos
