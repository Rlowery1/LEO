# Copilot Instructions — LEO (UE 5.7 AAA_Traffic)

These instructions are **non-negotiable**. They are derived from `System.md` in this repo and exist so Copilot follows the same contract automatically.

Canonical contract source: `System.md` (repo root).

## Authority & Verification
- Never claim a build/package is successful unless verified by CI logs/artifacts.
- Use these statuses:
  - **VERIFIED**: CI exit code == 0 and no fatal/compiler errors in artifacts
  - **FAILED**: CI exit code != 0 or compiler errors in artifacts
  - **UNVERIFIED**: CI not executed or no artifacts available

## Response Protocol (Required)
Before finalizing any implementation response, internally simulate these passes:

- **PASS 1 — ARCHITECT**: validate structural fit, module impact, acceptance criteria
- **PASS 2 — IMPLEMENTER**: minimal diffs only; avoid unrelated refactors
- **PASS 3 — REVIEWER**: check reflection macros, module boundaries, determinism, perf safety, marketplace compliance
- **PASS 4 — OPERATIONS**: predict CI impact; assign **VERIFIED / FAILED / UNVERIFIED**

Final output must contain the following sections (even if brief):

- Summary
- Assumptions (if any)
- File-by-file patch (what changed, where)
- CI Impact
- Verification Status (**VERIFIED / FAILED / UNVERIFIED**)

## Structural Boundaries (Repo Layout)
- Treat this layout as authoritative:
  - `HostProject/HostProject.uproject`
  - `Plugins/AAA_Traffic/AAA_Traffic.uplugin`
  - `Plugins/AAA_Traffic/Source/`
  - `.github/workflows/`
- Do **not** invent modules/folders or casually rename public API symbols.
- Do not modify `.uplugin` structure without clear justification.

## Workflow (Local Dev + Export)

This repo (`PublicRepoExport/`) is the published GitHub repo snapshot.
It is the source of truth for what gets pushed and what CI builds.

Local development may happen in the larger outer Unreal workspace (with extra local-only plugins).
In that case, use this workflow to avoid mixing:

1) Develop + integration-test in the outer project (`AAA_Traffic.uproject`) if it needs extra plugins.
2) Export/copy only the shippable plugin changes into this repo under `Plugins/AAA_Traffic/`.
3) CI validation runs by building `HostProject/HostProject.uproject` in a clean environment.

Rule:
- Treat `HostProject` as the clean build harness (CI). Treat the outer project as the integration sandbox.

## Engineering Requirements (UE 5.7+)
- Core simulation/runtime logic must be in **C++**.
- Blueprint is allowed only for configuration surfaces and high-level integration.
- Maintain strict Runtime vs Editor isolation:
  - Runtime modules must not include editor headers or depend on editor modules.
  - Guard editor-only code with appropriate `WITH_EDITOR` patterns.
- Keep `*.Build.cs` dependencies minimal and purposeful.

## Determinism Requirements
- With the same seed + initial state + config, the system must produce identical decisions.
- Randomness:
  - Forbidden: `FMath::Rand()` family functions, unseeded randomness.
  - Required: `FRandomStream` with an explicit, configurable/stable seed.
- Frame independence:
  - Avoid frame-rate dependent decision logic.
  - Prefer fixed-step simulation for behavior-critical logic; clamp `DeltaSeconds` when used.
- Deterministic selection:
  - Sort candidates deterministically.
  - Use explicit tie-break rules.
  - Never rely on container traversal order.
- Physics boundary:
  - Decision logic must remain deterministic even if Chaos physics isn’t.
  - Do not claim deterministic physics across machines without proof.

## Marketplace/Packaging Compliance
- Plugin must not require proprietary packs or hardcode asset references outside its scope.
- No hidden network operations.
- Do not depend on generated folders: `Binaries/`, `Intermediate/`, `DerivedDataCache/`, `Saved/`.
- Avoid breaking serialization/config without an explicit migration note.

## Change Discipline
- Minimal diffs; no broad refactors unless explicitly requested.
- If CI fails: fix the **first** compiler error with the smallest change; avoid rewrites.

## Code Hygiene (Non-Negotiable)
- **One implementation per role.** There must be exactly one class for each subsystem role (e.g., one `ITrafficRoadProvider`). Never create a second class that does the same job — extend or replace the existing one.
- **No dead code.** If code is superseded, delete it in the same change. Source control has the history.
- **Module inventory is locked.** The plugin has exactly two modules: `AAA_Traffic` (core runtime) and `AAA_TrafficEditor` (editor-only). Do not add new modules without explicit user approval.
- **No copy-paste utilities.** Shared functions live in one place. Do not duplicate a function into a second file and rename it to avoid collisions.

## Completion Criteria
A task is DONE only when:
- Patch delivered
- CI green
- No determinism violations introduced
- No runtime/editor boundary violations
- No packaging regressions introduced

Otherwise status remains **UNVERIFIED** or **FAILED**.
