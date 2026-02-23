# LEO — Unreal Engine 5.7 Traffic Engine Operational Contract

Scope:
- Repo: LEO
- HostProject/HostProject.uproject
- Plugins/AAA_Traffic/
- UE 5.7+
- GitHub Actions (self-hosted Windows runner)

This document defines non-negotiable assistant behavior.
This is enforcement logic, not guidance.

============================================================
SECTION 1 — AUTHORITY & TRUTH MODEL
============================================================

1.1 Machine Authority

The only accepted verification sources are:

- GitHub Actions run result (success/failure)
- Build exit code from CI
- Uploaded CI build artifacts/logs
- Structured logs produced by deterministic automation scripts

Verification statuses:

VERIFIED
  - CI exit code == 0
  - No fatal or compiler errors in artifact logs

FAILED
  - CI exit code != 0
  - Compiler errors present

UNVERIFIED
  - CI has not executed
  - No artifacts available

The assistant must never:
- Claim build success without CI confirmation
- Infer success from code inspection
- Assume compilation succeeded

============================================================
SECTION 2 — STRUCTURAL BOUNDARIES
============================================================

2.1 Repository Layout Awareness

Authoritative structure:

HostProject/
  HostProject.uproject

Plugins/
  AAA_Traffic/
    AAA_Traffic.uplugin
    Source/

.github/workflows/

The assistant must not:
- Invent modules
- Invent folder layouts
- Rename public API symbols casually
- Modify .uplugin structure without justification

============================================================
SECTION 3 — ENGINEERING REQUIREMENTS (UE 5.7+)
============================================================

3.1 C++ Core Mandate

All core simulation logic must be implemented in C++.

Blueprint exposure is allowed only for:
- Configuration surfaces
- High-level control interfaces
- Components for integration

Heavy runtime logic in Blueprint is prohibited unless explicitly requested.

3.2 Runtime / Editor Isolation

Runtime modules must not:
- Include editor headers
- Depend on editor modules
- Allow WITH_EDITOR code paths to execute in runtime builds

Editor functionality must be isolated or explicitly guarded.

3.3 Build.cs Discipline

- Minimal module dependencies only
- No unnecessary public dependencies
- Clear separation of runtime and editor

============================================================
SECTION 4 — TRAFFIC DETERMINISM ENFORCEMENT
============================================================

4.1 Reproducibility Requirement

Given:
- Same seed
- Same initial world state
- Same config

The traffic system must produce identical decisions.

4.2 Randomness Restrictions

Forbidden:
- FMath::Rand() and related functions usage
- Unseeded randomness

Required:
- FRandomStream with explicit seed source
- Seed must be configurable or stable
- Decisions must include deterministic tie-breaking

4.3 Frame Independence

Forbidden:
- Frame-rate dependent logic
- Per-frame probability checks
- Unbounded DeltaSeconds behavior

Required:
- Fixed-step simulation patterns when behavior-critical
- DeltaSeconds clamping where used
- Explicit time accumulation logic when applicable

4.4 Deterministic Selection

Whenever selecting between options:
- Sort candidates deterministically
- Define stable tie-break rules
- Never rely on container traversal order

4.5 Physics Boundary

Traffic decision logic must remain deterministic even if physics is not.

Never claim deterministic physics resolution across machines without proof.

============================================================
SECTION 5 — MARKETPLACE-LEVEL COMPLIANCE
============================================================

5.1 Plugin Independence

The plugin must:
- Not require proprietary content packs
- Not hardcode asset references outside its scope
- Not perform hidden network operations

5.2 Public API Stability

The assistant must not:
- Rename public UCLASS/USTRUCT casually
- Modify serialization layout without warning
- Break config keys without explicit migration note

5.3 Packaging Cleanliness

The assistant must not depend on:
- Binaries
- Intermediate
- DerivedDataCache
- Saved

Only source-of-truth content is considered authoritative.

============================================================
SECTION 6 — MULTI-PASS INTERNAL REVIEW PROTOCOL
============================================================

Every implementation response must internally simulate the following passes:

PASS 1 — ARCHITECT
- Validate structural fit
- Identify module impact
- Define acceptance criteria

PASS 2 — IMPLEMENTER
- Produce minimal diffs only
- Avoid unrelated refactors

PASS 3 — REVIEWER
Validate:
- Reflection macros correctness
- Module boundaries
- Determinism compliance
- Performance safety
- Marketplace compliance

PASS 4 — OPERATIONS
- Predict CI impact
- Define verification status
- Assign VERIFIED / FAILED / UNVERIFIED

Final output must contain:

- Summary
- Assumptions (if any)
- File-by-file patch
- CI Impact
- Verification Status

============================================================
SECTION 7 — FAILURE RESPONSE PROTOCOL
============================================================

If CI fails:

1. Identify first compiler error from artifact logs.
2. Fix smallest surface required.
3. Avoid speculative system rewrites.
4. Produce minimal corrective patch.
5. Await next CI result.

No architectural rewrites mid-debug unless explicitly instructed.

============================================================
SECTION 8 — SCOPE CONTROL
============================================================

The assistant must not:

- Perform broad refactors without request
- Introduce new subsystems without justification
- Change architecture direction silently
- Add complexity for aesthetic reasons

Minimalism is enforced.

============================================================
SECTION 9 — COMPLETION CRITERIA
============================================================

A task is considered DONE only when:

- Patch delivered
- CI green
- No determinism violations introduced
- No runtime/editor boundary violations
- No packaging regressions introduced

Otherwise status remains UNVERIFIED or FAILED.
