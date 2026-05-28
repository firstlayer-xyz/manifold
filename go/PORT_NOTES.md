# Go port — running notes

Things to revisit. Each entry: what / why / where it bites / when to fix.

## Fidelity audit — known deviations from C++

This section catalogs every place the Go port diverges from C++ at
the **algorithm / call-site / threshold / atomic semantics** level.
"Runtime differences" (goroutines instead of TBB threads,
`sync/atomic` instead of `std::atomic`) are NOT deviations as long
as the algorithmic call-site shape matches.

### Algorithmic deviations (results may differ in edge cases)

- **`SpectralNorm` uses closed-form symmetric-3x3 eigenvalue formula
  instead of the full Jacobi SVD3x3** (`internal/geom/spectral_norm.go`).
  - **C++:** `src/svd.h` runs a full 304-line Jacobi-iteration SVD3x3
    and returns the largest singular value.
  - **Go:** Smith (1961) closed-form formula:
    sqrt(largestEigenvalue(A^T · A)) via the analytic cubic-root
    formula for a 3x3 symmetric matrix.
  - **Mathematically equivalent** to floating-point precision but
    NOT the same code path. Numerical edge cases (near-singular A)
    may differ at the last few ulps.
  - **Fix when:** SVD3x3 gets ported for other reasons (currently no
    other Go caller needs the full SVD).

- **`internal/parallel.StableSort` uses chunk-sort + log-P pairwise
  stable merge instead of TBB's `merge_sort` recursion.**
  - C++ uses `tbb::parallel_for` + `parallel_invoke` recursion in
    `src/parallel.h details::mergeRec`.
  - Go: each worker sorts its chunk with `sort.SliceStable`, then a
    log-P pairwise pass merges chunks of doubling width.
  - Same output ordering; different memory access pattern.

- **`internal/parallel.InclusiveScan` / `ExclusiveScan` use a 2-pass
  parallel prefix sum** instead of TBB's `parallel_scan`.
  - Same algorithmic shape (Hillis-Steele family).

### `std::map` (ordered) vs Go `map` (randomized iteration)

C++ `std::map<K,V>` iterates in sorted key order; Go `map[K]V`
iteration is **randomized per run**. When porting C++ code that
iterates a `std::map`, the port must preserve sorted-key iteration.

**Use the `internal/orderedmap` package** for any future port
that mirrors a C++ `std::map`. It provides
`OrderedMap[K cmp.Ordered, V any]` with `Set`/`Get`/`Delete`/
`Clear`/`Contains`/`Len`/`Keys`/`All` — sorted-key iteration via
the modern `iter.Seq2[K, V]` range-over-func.

Audit of currently drilled code: clean. The only `std::map`
exposed to drilled code is `meshIDtransform`, and the bridge
`MeshIDTransforms()` accessor flattens it to a sorted slice
before crossing into Go (so iterating the returned slice is
already in sorted order). When the bridge goes away, the Go-side
storage for `meshIDtransform` will become an
`orderedmap.OrderedMap[int32, MeshIDRelation]`.

C++ files using `std::map` to watch on future drills:
- `impl.cpp::IncrementMeshIDs` — iterates `meshIDtransform` to
  assign new IDs; ordering affects the assignment.
- `smoothing.cpp::CreateTangents` — `meshIDtoNormalTransform`
  (currently lookup-only in the C++ but verify on port).
- `polygon.cpp::Triangulate` — `hole2BBox_`.
- `boolean_result.cpp::Boolean3` — `concurrent_map` (which is
  `std::map` under MANIFOLD_PAR=0).

Plain Go `map[K]V` iteration is acceptable when:
- It's used only for lookups (`m[k]` access).
- It's used only for membership checks (sets).
- It's only iterated to clear (delete every key).
- The C++ counterpart is `std::unordered_map` / `unordered_set`
  (then both languages have implementation-defined order — fine
  as long as the algorithm's output is a SET whose ordering
  doesn't matter, e.g. Slice/Project polygon-list output).

### `HashTable[V]` Lookup vs Slot

C++ `HashTableD::operator[]` returns a `V&` — caller works with a
reference into `values_` and is responsible for any concurrent
read/write coordination. The Go port exposes both:

- `Lookup(key) (V, bool)` — copies the value. Race-detector trips
  on concurrent reads (even of fields the caller doesn't use) if
  another goroutine writes any field of that V.
- `Slot(key) (*V, bool)` — returns a pointer, no copy. Caller
  reads only the specific fields they need; for fields that are
  written concurrently, caller uses `sync/atomic`. Used by the
  LevelSet `ComputeVerts` kernel to read `distance` / `movedVert`
  (sequenced-before by NearSurface) while concurrent workers write
  `edgeVerts` atomically.

This mirrors the C++ benign-race pattern under the stricter Go
memory model. The C++ relies on word-sized non-atomic reads being
"benign-enough"; Go requires atomic field accesses.

### Two-arg `CreateHalfedges` is still bridge

- The single-arg form is fully drilled (both sort and bucket
  branches, the duplicate-detection body lambda, and the parallel
  outer-body loop variant).
- The two-arg form (`triProp` + `triVert` separately, used by
  `Manifold(MeshGL)` ingest with merged-vert input) is still
  bridge-routed because no Go caller exercises it yet.

### Ephemeral Collider vs persistent `collider_`

- C++ keeps `collider_` as an `Impl` member field built once by
  `SortGeometry` and reused by Slice / RayCast / MinGap /
  Minkowski / IsSelfIntersecting / Boolean3.
- Go builds a fresh Collider inside each `Slice`,
  `IsSelfIntersecting`, and `MinGap` call via
  `collider.New(i.GetFaceBoxMorton())`. Algorithm at the call site
  is the same; only the persistence is missing.
- **Leaf index == face index, no remap.** This mirrors C++ exactly:
  `GetFaceBoxMorton` returns the two parallel `faceBox`/`faceMorton`
  arrays (length `NumTri`, uncompacted, `NoMortonCode` + empty box
  for removed tris), and the recorder closures use the `tri`
  indices directly — just like the C++ recorders index `vertPos_`
  via `halfedge_.Start(3 * tri + j)`.
- **Precondition (same as C++ `collider_` validity):** the Impl's
  faces are Morton-sorted (`SortGeometry`/`SortFaces` physically
  gathered them) and `bBox_` is stable, so recomputing the Morton
  codes reproduces the ascending order the `Collider` ctor requires.
  The normal construction path guarantees this — `CalculateBBox`
  sets `bBox_` to the tight vert bbox before `SortGeometry`, and
  `collider_.GetBoundingBox()` returns that same tight bbox after,
  so there is no drift.
- **History:** an earlier port computed a *compacted*
  `perFaceBoxMorton` (skipping removed tris) and then externally
  `StableSort`ed + carried a `sortedFaceID` remap — a workaround
  for an incompatible container that diverged from the C++ form.
  Removed; `GetFaceBoxMorton` now matches the C++ two-vector
  signature and both Impl/MutableImpl expose it.
- **Why no persistence yet:** the facade types (`manifold.Impl`,
  `manifold.MutableImpl`) are thin wrappers around bridge handle
  pointers; each `getImpl(m)` returns a fresh wrapper.
- **Fix when:** persistent Collider home is built (either inside
  `bridge.Impl` with a Go finalizer or a
  `sync.Map[bridgeHandlePtr, *Collider]` sidecar).

### `Impl::Transform` — full Collider rebuild

- C++ `Impl::Transform` updates `collider_` via
  `Collider::Transform` (axis-aligned) or `Collider::UpdateBoxes`
  (otherwise) — both reuse the existing radix tree.
- Go uses the full bridge `BuildCollider` rebuild because (a) the
  Go Collider isn't persisted and (b) the bridge doesn't expose
  those fast-paths.
- Algorithmically different — the radix tree is rebuilt from
  scratch instead of transformed in place.
- **Fix when:** Go-side Collider gets a persistent home AND bridge
  Collider::Transform / UpdateBoxes accessors are exposed.

### `Manifold::Transform` bypasses CSG lazy fusion

- C++ `Manifold::Transform` defers via `CsgNode::Transform` so
  chained transforms fuse into a single matrix before evaluation.
- Go wires `Manifold::Transform` (and `Translate`/`Scale`/`Mirror`/
  `Rotate`) directly to `Impl::Transform`, eager per call.
- **Fix when:** CSG tree (`CsgNode` / `CsgLeafNode` / `CsgOpNode`)
  is drilled. The C++ Boolean3 engine is the gating dependency.

## Algorithms still in C++ (bridge passes through)

- `SubdivideN`, `RefineN`, `RefineToLength`, `RefineToTolerance`
  — `src/subdivision.cpp` (~800 lines).
- `SimplifyTopology` — `src/edge_op.cpp` (~800 lines).
- `CreateTangents` (`CreateTangentsIdx` and
  `CreateTangentsFromSmoothness`) — `src/smoothing.cpp`, depends on
  `GetNormal`, `TangentFromNormal`, `CircularTangent`,
  `IsInsideQuad` plus the Bezier/quaternion machinery in
  `InterpTri`. (ForVert is drilled.)
- `RayCast`, `Minkowski` — still C++; use the bridge Collider.
  When drilled, can use the Go `internal/collider` package directly.
  (`MinGap` is now drilled — native Go via `internal/collider` +
  `geom.DistanceTriangleTriangleSquared`.)
- `BuildCollider` (the bridge call that keeps the C++ Impl's
  `collider_` populated) survives only for the unported C++-side
  algorithms above.
- `Boolean3` / `CSG tree` / `LoadPNode` / `NewCsgOpNode` —
  multi-KLOC.
- `Triangulate` (Earcut/CDT, `src/polygon.cpp` ~1000 lines) —
  called from `extrude`'s cap-triangulation step.
- `Manifold(MeshGL{,64})` constructor — ~200 lines plus
  `RemoveDegenerates` (which depends on `CollapseShortEdges` +
  `SwapDegenerates`, both still bridge). Drilled so far on this
  chain: `IsManifold`, `Is2Manifold`, `RemoveUnreferencedVerts`,
  `DedupePropVerts`, `SplitPinchedVerts` (both branches),
  `DedupeEdges` (both branches + map fallback) +
  `dedupeEdge`/`updateVert`/`pairUp`, `CleanupTopology`. Still
  bridge: `CollapseShortEdges`, `SwapDegenerates` (inside
  `RemoveDegenerates`), plus the MeshGL ingest entry point itself.

## Cross-cutting deferred work

- **`Quality` and `DisjointSets` are independent Go state.** They
  do not share with the C++ side. Fine while the C++ Manifold is
  the black box; revisit only if we ever want one process to
  mutate Quality from C++ and read it from Go (or vice versa).
- **`ReserveIDs` still uses the bridge atomic.** Splitting it
  would cause meshID collisions between Go-allocated and
  C++-allocated manifolds in the same process. Keep as bridge
  until everything else is ported.

## Parallel primitives (`internal/parallel`)

Every existing Go port uses the `internal/parallel` package, which
mirrors `src/parallel.h`:

- `ExecutionPolicy` (Par/Seq), `AutoPolicy(size, threshold)` —
  same semantics as C++ `autoPolicy`, default threshold
  `kSeqThreshold = 1e4`.
- `ForEachN(policy, n, f)` — C++
  `for_each_n(policy, countAt(0), n, f)`.
- `Reduce[T]`, `TransformReduce[S,T]` — chunked partial fold +
  serial combine.
- `AllOf`, `AnyOf` — workers + atomic stop flag for early exit.
- `CountIf` — per-worker counts + serial sum.
- `Transform`, `Fill`, `Sequence`, `Gather`, `Scatter`, `Permute`
  — wrappers around `ForEachN`.
- `InclusiveScan`, `ExclusiveScan` — two-pass parallel prefix scan.
- `CopyIf` — three-pass parallel filter.
- `StableSort` — chunk-sort + log-P pairwise stable merge.
- `AtomicAddFloat64` — CAS loop over the bit pattern, mirroring
  `src/utils.h::AtomicAdd<double>`.

Threshold matching with C++ — the default thresholds used by the
no-policy form of each C++ primitive:

| Primitive               | C++ default threshold |
| ----------------------- | --------------------- |
| `for_each_n`            | `1e4` (kSeqThreshold) |
| `reduce`/`transform_reduce` | `1e5`              |
| `all_of`/`any_of`       | `1e5`                 |
| `count_if`              | `1e4`                 |
| `inclusive_scan`/`exclusive_scan` | `1e5`       |
| `transform`             | `1e5`                 |
| `fill`                  | `5e5`                 |
| `gather`/`scatter`/`sequence` | `1e5`           |
| `copy`                  | `1e6`                 |
| `stable_sort`           | `1e4`                 |

**Audit rule when porting**: at every C++ call site, the Go port
must use the same primitive AND the same threshold. When C++ uses
the no-policy form, the Go port uses `parallel.AutoPolicy(n, T)`
with `T` from the table above. When C++ uses an explicit policy
(e.g. `autoPolicy(n, 1e5)`), Go matches that explicit value. A
"stable_sort with outer policy 1e5" was a bug pattern fixed during
the fidelity audit — `stable_sort` uses its own 1e4 default
independent of any surrounding `for_each_n` policy.

Atomic shared-write sites that the C++ `std::atomic` paths require
(`keep[startVert]` in RemoveUnreferencedVerts/CompactProps,
`vertHalfedgeMap[v]` atomic-min in CalculateVertNormals,
`counters` in CalculateCurvature, per-vert curvature accumulators,
`largestEdge[vert]` in `SplitPinchedVerts` parallel branch, bucket
offsets in `CreateHalfedges` bucket branch) use `sync/atomic` with
the same semantic. The C++ `std::atomic::compare_exchange_strong`
pattern that returns the failing value is replicated as
`CompareAndSwapInt64` + a fresh `LoadInt64` on failure (Go's CAS
doesn't return the old value).

The full retrofit passes under `go test -race ./...`.

## Naming / structure

- **Bridge types named identically to facade types** (`bridge.Impl`
  vs `manifold.Impl`). Different packages, no collision, but a
  reader skimming code might wonder. Acceptable; collapses when
  the bridge goes away.
- **`bridge.Smoothness` leaks through `CreateTangentsFromSmoothness`
  signature.** Consider a manifold-package `Smoothness` type alias.
