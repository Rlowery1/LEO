# RoadBLD (UHT Reflection) — Discovery Report (Evidence Baseline)

Status: COMPLETE (reflection-level)

This report records what is *provably present* in the RoadBLD plugin **via UnrealHeaderTool (UHT) generated reflection artifacts** available in this workspace, and what was *not found* after directory inventory + targeted keyword scans.

It exists to enforce the discovery lock in System.md: downstream architecture decisions must not assume semantics that are not evidenced.

## Scope & Constraints

- Host workspace: `AAA_Traffic` (UE 5.7+ host project)
- RoadBLD authored `.h/.cpp` are not available in this export.
- Evidence source-of-truth for this report:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/*`

Notes:
- This report intentionally avoids copying large proprietary code blocks. It summarizes reflected symbols, data shapes, and high-signal comments at a conceptual level.

## Evidence Method

1) Enumerated all files in the UHT output directory.
2) Ran broad keyword scans over the full UHT directory for:
   - lane-to-lane routing primitives (connector/link/from/to)
   - traffic-law semantics (stop/yield/signal/priority/right-of-way/speed)
   - intersection/junction/connectivity terms
3) Deep-read the UHT outputs for the highest-signal classes/structs:
   - dynamic road network connectivity
   - corner/intersection representation
   - lane and edge curve APIs
   - road utility library helpers

## Proven Concepts (Reflected Surfaces)

### 1) Connectivity Exists at the “Corner on Edge Curves” Level

**Canonical connectivity datum:** `FRoadNetworkCorner`

What’s evidenced:
- A corner connection references *edge curves* and *distances along those curves*.
- A corner has a stable identifier (GUID-style) used to preserve edits across rebuilds.
- Corners appear to be the bridge between:
  - road network topology (which edge connects to which)
  - intersection geometry (intersection point, directions, optional corner curve)
  - perimeter-cut and mask generation (cut indices and mask arrays)

Primary evidence:
- `FRoadNetworkCorner` reflection in:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/ClothoidCurve.gen.cpp`

**Traversal API:** edge+distance → previous/next corner

What’s evidenced:
- `ADynamicRoadNetwork` exposes corner traversal functions operating on `(UEdgeCurve*, double DistanceAlongEdge)` returning an `FRoadNetworkCorner` (with a found flag).
- Semantics are explicitly documented as “next/previous” ordered by distance *along the same start edge*.

Primary evidence:
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadNetwork.generated.h`
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadNetwork.gen.cpp`

### 2) Network Stores Corners, Perimeter Cuts, and Masks (with Edit Preservation)

What’s evidenced in `ADynamicRoadNetwork`:
- Storage arrays exist for:
  - road network corners
  - perimeter cuts
  - intersection masks
  - a separate corner-edit persistence buffer (to preserve edits across rebuilds)
- Deprecation/migration exists around walkways vs roads stored in-network (walkways appear to be represented as roads with a `RoadType` distinction).

Primary evidence:
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadNetwork.gen.cpp`

### 3) Manual Intersections Exist and Store Corner Connections

What’s evidenced:
- A manual intersection actor stores an internal list of `FRoadNetworkCorner` entries.
- Helpers exist to add/remove/clear and count connections.

Primary evidence:
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/CustomIntersection.generated.h`
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/CustomIntersection.gen.cpp`

### 4) Roads Have Lanes (Geometry), but Routing Is Not Reflected

What’s evidenced:
- Roads can enumerate lanes and edges.
- Lanes exist as curve objects and support regeneration/offset point updates.
- Road utilities can select a lane by world location and fetch lanes “outside” a lane (editor-style workflow helpers).

Primary evidence:
- Road lane object (geometry-focused):
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadLane.generated.h`
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadLane.gen.cpp`
- Road actor lane enumeration:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoad.gen.cpp`
- Lane selection helpers:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/RoadUtilityLibrary.gen.cpp`

### 5) Turn Metadata Exists as Authoring Geometry, Not Legality

What’s evidenced:
- “Turn radius” exists as control-spline metadata for road authoring.
- This is *not* the same thing as “turn restrictions” or lane-to-lane permitted turns.

Primary evidence:
- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoad.gen.cpp`

### 6) “Traffic-ish” Features Are Visual/Automation-Oriented

What’s evidenced:
- Crosswalk actor exists.
- Road sign actor exists (prefab/visual text entries).
- Automation parameters exist for:
  - crosswalk creation between intersection corners
  - “right/left/straight only” lane decals
  - solid lane marking behavior near intersections

Primary evidence:
- Crosswalk actor:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/Crossing.gen.cpp`
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/Crossing.generated.h`
- Signage:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/RoadSign.gen.cpp`
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/RoadSign.generated.h`
- Automation params:
  - `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/DynamicRoadData.gen.cpp`

## Not Found (After Global UHT Scans)

The following concepts were **not found** as reflected APIs/types/keywords in the scanned UHT output:

- Lane-to-lane routing graph primitives:
  - explicit “from lane / to lane” connectors
  - lane transition tables at intersections
  - routing node/edge types for driving policy
- Traffic control / legality model:
  - traffic light/signal state machines
  - stop/yield/right-of-way priority rules
  - speed limit primitives

Interpretation constraints:
- Absence in reflection does *not* prove the implementation doesn’t exist in authored code.
- It *does* mean: **we must not rely on it** for the adapter without additional evidence.

## Determinism & Stable-ID Observations

What’s evidenced:
- Corner connections carry stable identifiers intended to survive rebuilds.
- Comments indicate avoidance of array-index coupling and use of edit buffers for persistence.

Implication:
- Adapter output must be keyed by stable IDs (or derived stable keys), never by transient array indices.

## Adapter Implications (What We Can Safely Export)

Given current evidence, a “press one button” adapter can deterministically export:

1) **Edge graph with corner events**
   - For each road edge curve: ordered corner connections by distance
   - Corner metadata: start/end edge, start/end distance, intersection point, direction hints

2) **Lane geometry sampling surfaces**
   - Enumerate lanes per road
   - Sample lane curves/offset points for drivability geometry
   - Use lane selection helpers when needed (e.g., debug, editor tooling)

3) **Intersection geometry artifacts**
   - Perimeter cuts + masks where present (for spatial reasoning / nav mesh hints)

What we cannot assert (yet):
- Which lane turns into which lane through a corner.
- Any legal priority or signal-driven behavior.

## Open Questions (Requires More Evidence Than UHT Provides Here)

- Is there an unreflected lane-connector model in authored sources?
- If not, what deterministic inference rules are acceptable for generating lane-to-lane connectors from:
  - corner directions
  - lane centerline proximity
  - edge ordering
  - road type and lane side/index

## UHT Directory Inventory (for Audit)

The following key files were present and scanned (non-exhaustive listing):

- `DynamicRoadNetwork.*`, `CustomIntersection.*`, `DynamicRoad.*`, `DynamicRoadLane.*`, `EdgeCurve.*`
- `RoadUtilityLibrary.*`, `RoadControlSplineComponent.*`, `RoadBLDTypes.*`, `DynamicRoadData.*`
- `Crossing.*`, `RoadSign.*`

Full inventory can be reproduced by listing:

- `Plugins/RoadBLD/Intermediate/Build/Win64/UnrealGame/Inc/RoadBLDRuntime/UHT/`
