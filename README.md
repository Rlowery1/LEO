# AAA_Traffic (Unreal Engine Plugin)

Public, code-only plugin repo.

## Repo layout

- `Plugins/AAA_Traffic/` — the plugin source
- `HostProject/` — minimal Unreal project used to build/validate the plugin (useful for CI)

## Requirements

- Unreal Engine 5.7 (or compatible)

## Local build (manual)

Open `HostProject/HostProject.uproject` in Unreal Editor and let it compile.

> Note: This repo intentionally excludes all `.uasset`/`.umap` content and any third-party plugins/assets.
