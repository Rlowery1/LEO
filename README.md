# AAA_Traffic (Unreal Engine Plugin)

Public, code-only plugin repo.

## Repo layout

- `Plugins/AAA_Traffic/` — the plugin source
- `HostProject/` — minimal Unreal project used to build/validate the plugin (useful for CI)

## Requirements

- Unreal Engine 5.7 (or compatible)

## CI (GitHub Actions)

This repo is set up to build on a **self-hosted Windows runner** with UE 5.7 installed.
The workflow expects UE at `C:\\Program Files\\Epic Games\\UE_5.7` (see `.github/workflows/build-plugin.yml`).

### dev vs main

- Pushes to `dev` and `main` both run the build.
- The build output is uploaded as a GitHub Actions artifact named like `AAA_Traffic_Win64_dev` or `AAA_Traffic_Win64_main`.
- Pull requests only run on the self-hosted runner if the PR comes from a branch inside this same repo (fork PRs are skipped for safety).

## Local build (manual)

Open `HostProject/HostProject.uproject` in Unreal Editor and let it compile.

> Note: This repo intentionally excludes all `.uasset`/`.umap` content and any third-party plugins/assets.
