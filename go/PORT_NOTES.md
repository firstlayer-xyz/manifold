# Go port — running notes

Things to revisit. Each entry: what / why / where it bites / when to fix.

## Algorithm shortcuts (different strategy, same output)

- **Slice uses linear triangle scan** (`impl_slice.go`).
  - **Why:** C++ uses the Collider's bounding-box query to filter
    candidate triangles in O(log N + k). The Collider is not ported.
  - **Bites:** O(N) per slice instead of O(log N). Acceptable on small
    meshes; significant cost on large ones.
  - **Fix when:** the Collider is ported to Go.

- **CreateHalfedges panics on duplicate triangles** (`impl_halfedges.go`).
  - **Why:** The C++ "body" lambda handles duplicates via reordering
    and the removed[] mask. The Go port covers only the no-duplicate
    branch (which is all our primitive shape tables hit).
  - **Bites:** panic for any caller that supplies a triVerts list with
    two triangles sharing the same (start, end, third-vert) tuple.
  - **Fix when:** the next caller would actually hit this — e.g.
    Manifold(MeshGL) ingest on a degenerate-input mesh.

## Algorithms still in C++ (bridge passes through)

- `SubdivideN`, `RefineN`, `RefineToLength`, `RefineToTolerance`
  — `src/subdivision.cpp` (~800 lines).
- `SimplifyTopology` — `src/edge_op.cpp` (~800 lines).
- `Hull` (Quickhull3D) — `src/quickhull.cpp` (~865 lines).
- `SetNormals` — `src/smoothing.cpp` (~185 lines + helper cascade).
- `CreateTangents` (`CreateTangentsIdx` and `CreateTangentsFromSmoothness`)
  — `src/smoothing.cpp`, depends on `ForVert`, `GetNormal`,
  `TangentFromNormal`, `CircularTangent`, `IsInsideQuad`.
- `RayCast`, `MinGap`, `Minkowski` — all use the C++ `Collider`.
- `BuildCollider` (the post-vert tail of `SortGeometry`) — AABB tree
  construction, the only C++ piece remaining in `newImplFromShape`.
- Boolean3 / CSG tree / `LoadPNode` / `NewCsgOpNode` — multi-KLOC.
- `LevelSet` — marching-tetrahedra, currently invoked via a Go-side
  SDF callback trampoline.
- `Triangulate` (Earcut/CDT, `src/polygon.cpp` ~1000 lines)
  — called from `extrude`'s cap-triangulation step.
- `Manifold(MeshGL{,64})` constructor — ~200 lines plus
  `IsManifold`, `CleanupTopology`, `DedupePropVerts`,
  `RemoveDegenerates`, `RemoveUnreferencedVerts`.

## Cross-cutting deferred work

- **Two-arg form of `CreateHalfedges`** (`triProp`, `triVert` distinct).
  - **Why:** Used by `Manifold(MeshGL)` ingest when the input has
    merged verts. The one-arg form is drilled; the two-arg form
    still routes through the bridge wrapper (no Go caller right
    now since `Manifold(MeshGL)` ingest is not drilled).
- **`Impl::IsManifold` and friends.** Not exposed at the public API,
  but used internally by `Manifold(MeshGL)` ingest and by C++ debug
  assertions. Drill when ingest is drilled.
- **`Quality` and `DisjointSets` are independent Go state.** They do
  not share with the C++ side. Fine while the C++ Manifold is the
  black box; revisit only if we ever want one process to mutate
  Quality from C++ and read it from Go (or vice versa).
- **`ReserveIDs` still uses the bridge atomic.** Splitting it would
  cause meshID collisions between Go-allocated and C++-allocated
  manifolds in the same process. Keep as bridge until everything
  else is ported.

## Naming / structure

- **Bridge types named identically to facade types** (`bridge.Impl`
  vs `manifold.Impl`). Different packages, no collision, but a
  reader skimming code might wonder. Acceptable; collapses when
  the bridge goes away.
- **`bridge.Smoothness` leaks through `CreateTangentsFromSmoothness`
  signature.** Consider a manifold-package `Smoothness` type alias.
