# About This Site

How this documentation site is built and maintained.

Built with [Zensical](https://zensical.org/), a modern documentation site generator.

## Stack

| Component | Tool |
|---|---|
| Framework | [Zensical](https://zensical.org/) |
| Diagrams | [Mermaid](https://mermaid.js.org/) |
| Hosting | [GitHub Pages](https://pages.github.com/) via GitHub Actions |
| Development | [Claude Code](https://claude.ai/) (AI-assisted) |

## Why Zensical over Astro

An earlier version of this site was built with [Astro + Starlight](https://zeulewan.github.io/robot-docs-astro/). Starlight is a great docs framework and even has an MCP server, but Zensical works better for agentic workflows — Claude Code can read, edit, and build the site entirely from the terminal without needing browser-based tooling or custom integrations.

## Source

[zeulewan/robot-docs](https://github.com/zeulewan/robot-docs) on GitHub. Content lives in `docs/` as Markdown files. Push to `main` triggers a GitHub Actions workflow that builds and deploys to Pages.
