# Copilot Instructions — LEO (UE 5.7 AAA_Traffic)

These instructions are **non-negotiable**. They are derived from `System.md` in this repo and exist so Copilot follows the same contract automatically.

## Authority & Verification
- Never claim a build/package is successful unless verified by CI logs/artifacts.
- Use these statuses:
  - **VERIFIED**: CI exit code == 0 and no fatal/compiler errors in artifacts
  - **FAILED**: CI exit code != 0 or compiler errors in artifacts
  - **UNVERIFIED**: CI not executed or no artifacts available

## Structural Boundaries (Repo Layout)
- Treat this layout as authoritative:
  - `HostProject/HostProject.uproject`
  - `Plugins/AAA_Traffic/AAA_Traffic.uplugin`
  - `Plugins/AAA_Traffic/Source/`
  - `.github/workflows/`
- Do **not** invent modules/folders or casually rename public API symbols.
- Do not modify `.uplugin` structure without clear justification.

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
  - Forbidden: `FMath::Rand*`, unseeded randomness.
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

## Completion Criteria
A task is DONE only when:
- Patch delivered
- CI green
- No determinism violations introduced
- No runtime/editor boundary violations
- No packaging regressions introduced

Otherwise status remains **UNVERIFIED** or **FAILED**.
