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

Audit of currently drilled code: clean.

- `meshIDtransform` is the only `std::map` exposed to drilled code;
  the bridge `MeshIDTransforms()` accessor flattens it to a sorted
  slice before crossing into Go (so iterating the returned slice is
  already in sorted order). When the bridge goes away, the Go-side
  storage becomes an `orderedmap.OrderedMap[int32, MeshIDRelation]`.
- `Project`'s `AssembleHalfedges` (src/face_op.cpp:41-67) uses a
  `std::multimap<int,int>` keyed on startVert. The Go port mirrors
  it with `orderedmap.OrderedMap[int32, []int]` (the value slice is
  the equal-key bucket), so `begin()` seeds each polygon from the
  lowest remaining startVert — faithful to the ordered container.
  (An earlier port used a plain Go `map` + random-key start, which
  diverged; fixed.)
- `Slice` (src/face_op.cpp:376) uses `std::unordered_set<int>`, so a
  plain Go `map[int]struct{}` is the faithful equivalent — both are
  unordered, and the output polygon order is unspecified in both.
  No orderedmap needed here.

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

### `CreateHalfedges` — both forms drilled

- `CreateHalfedges(triProp, triVert)` is fully native (both sort and
  bucket branches, the duplicate-detection body, and the parallel
  outer-body loop). `triVert` may be nil — that is the single-arg
  form (startVert == propVert == triProp). When `triVert` is present
  (the `Manifold(MeshGL)` needsPropMap path), startVert/endVert come
  from `triVert` and propVert from `triProp`, mirroring C++
  PrepHalfedges<false>. Differential-tested via the ingest corpus.

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

### `Impl::Transform` — Collider refresh (now faithful)

- C++ `Impl::Transform` updates `collider_` via
  `Collider::Transform` (axis-aligned) or `Collider::UpdateBoxes`
  (otherwise) — both reuse the existing radix tree, leaving the
  mesh's face order untouched.
- Go now mirrors this exactly: `result := i.Copy()` carries a copy
  of the source's C++ `collider_` (the copy ctor copies it), and
  the bridge fast-paths `ColliderTransform` /
  `ColliderUpdateBoxes` update it in place — `collider.IsAxisAligned`
  picks the branch.
- **History (fixed):** an earlier port called `SortFaces`
  (→ `GatherFacesInPlace`) before a full `BuildCollider` rebuild,
  which **physically Morton-reordered** the result's
  halfedges/triRef/normals/tangents — violating Transform's
  "preserves triangulation/halfedge ordering" contract. Removed.
- **Remaining deviation:** the collider still lives C++-side (the
  two new bridge fns), because the Go Collider isn't persisted on
  the bridge Impl. Goes away with the persistent-Collider home
  noted above.

### `SetProperties` nil-callback zero-fill — benign-race adaptation

- C++ runs a `Par` loop over triangles writing 0 into each
  referenced propVert slot (src/manifold.cpp:636-646). Adjacent
  triangles share propVerts, so concurrent threads write 0 to the
  same slot — a benign idempotent-zero race C++ tolerates.
- Go's `make([]float64, …)` already zero-initializes the buffer (as
  C++'s `Vec(…, 0)` does), so the fill is redundant; the nil case is
  a no-op. Replaying it as a parallel write would be a benign race
  that `-race` flags, so Go skips it. Output is identical. The
  callback case keeps C++'s `Seq` policy.

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
- `SimplifyTopology` — `src/edge_op.cpp`. PARTIALLY drilled:
  `RemoveDegenerates` and its whole collapse/swap core are now native
  Go (`flagStore`, `collapseEdge`+`collapseTri`/`removeIfFolded`/
  `formLoop`, `recursiveEdgeSwap`+`is01Longest`, `collapseShortEdges`,
  `swapDegenerates`), differential-tested vs the bridge. Still bridge:
  `SimplifyTopology` itself = `CollapseColinearEdges` (needs
  `MarkCoplanar`) on top of the `RemoveDegenerates` steps.
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
- `Triangulate` (`src/polygon.cpp`) — PARTIALLY DRILLED, in the
  `internal/triangulate` package. Native today (line-by-line ports):
  - `Triangulate` / `TriangulateIdx` / `TriangulateIdxHalfedges`
    dispatch (polygon.cpp:931-1017), `IsConvex` + `TriangulateConvex`
    zig-zag fan (172/195), `HalfedgeTriangulation` (polygon_internal.h).
  - The full `EarClip` ear-clipper, **including holes/key-holing**:
    ring construction + lifecycle (`Initialize`/`Link`/`Clipped`/`Loop`/
    `ClipEar`/`ClipIfDegenerate`), all predicates (`InsideEdge`/`IsConvex`/
    `IsReflex`/`SignedDist`/`Cost`/`DelaunayCost`/`EarCost`/`InterpY2X`),
    `FindStart` (Kahan/Neumaier area), `ProcessEar`, `VertCollider`,
    `TriangulatePoly` clip loop, and the key-holing path `CutKeyhole`/
    `FindCloserBridge`/`JoinPolygons` (CW holes bridged into a CCW outer).
    Two adversarial line-by-line audits (predicates+clip loop, then
    keyholing): 0 confirmed divergences. `Vert::Interior` (polygon.cpp:331)
    is dead code in C++ (zero call sites) and is intentionally not ported.
  - `std::multiset<VertItr, MinCost/MaxX>` replicated by
    `orderedMultiset` (AVL + insertion-seq FIFO tie-break), 50k-op fuzzed.
  - Differential-tested vs the bridge (`TestTriangulateConvex_VsCpp`,
    `TestTriangulateConcave_VsCpp`, `TestTriangulateHoles_VsCpp`): convex
    matches exactly; holes match exactly (set-equal) on every tested case;
    concave is validity-equivalent (valid triangulation, same triangle
    count) and matches as a SET where float allows — last-ULP `EarCost`
    differences can reorder near-equal-cost ears on symmetric inputs (C++
    TestPoly itself checks only the count). `triangulateNative`
    (impl_triangulate.go) is now **pure native — no bridge fallback**.
  - The 2D kd-tree (`tree2d.go`: `BuildTwoDTree`/`BuildTwoDTreeImpl`/
    `QueryTwoDTree`, a verbatim port of src/tree2d.{h,cpp}) backs the ear
    collider: `VertCollider` returns an `idxCollider{points, itr}`, builds
    the tree in place, and `EarCost` queries it by the ear's bounding box
    (recovering each vert via `itr[point.Idx]`). C++ `VecView` sub-views map
    to Go sub-slices (shared backing array, in-place `sort.SliceStable`).
    Replaced the earlier O(n^2) brute-force scan with the tree's O(n log n)
    build + O(log n + k) per-ear query; output-identical (all differential
    tests still pass). Adversarial line-by-line audit: 0 findings.
  - Wired into the cap triangulation of `extrude` (native
    `triangulate.TriangulateIdx(polygonsIndexed)`, mirroring C++ Extrude)
    and `revolve` (native `triangulate.Triangulate(polygons, -1)`,
    mirroring C++ Revolve where `epsilon_` is still its default of -1).
    Both iterate the `[][3]int` triples directly — the Go ≈
    `std::vector<ivec3>` — matching the C++ `for (const ivec3& tri : ...)`
    loops. No flatten-to-`[]int32` shim: the earlier `triangulateNative`
    adapter (which only existed to mimic `bridge.Triangulate`'s flat
    signature) was removed; the test-only `nativeFlat` helper does the
    flattening solely to diff against the bridge oracle.
  - Validated against the manifold polygon corpus (the same fixtures as
    the C++ PolygonTest: polygon_corpus.txt, sponge.txt, zebra.txt,
    zebra3.txt) via `TestTriangulateCorpus_VsCpp` — every entry matches
    the curated expected triangle count and the bridge count, across the
    Basic / Turn180 / Duplicate variants. zebra/zebra3 are gated behind
    `-short` (the increment-6 brute-force collider is O(n^2)).
- `Manifold(MeshGL{,64})` constructor — FULLY DRILLED. The native ingest
  (`validateMeshGL` + `newImplFromMeshGL`, `impl_meshgl_ingest.go`) ports
  the entire C++ constructor body (src/impl.h:277-504): validation cascade,
  prop2vert merge, dual-stride vert/prop split, run handling (triRef +
  meshIDtransform), triProp/triVert build + degenerate cull, the (now
  native) two-arg CreateHalfedges, and the finalize tail. Generic over
  float32/float64; `useSingle` threads into SetEpsilon. The needsPropMap
  case (extra props + vertex merge) uses the native two-arg CreateHalfedges
  — no bridge fallback remains. Differential-tested vs the bridge
  (`TestMeshGLIngest_VsCpp` incl. props/needsPropMap, `_Float32_VsCpp`).
  Permitted bridge touch-points (shared with every other drilled step):
  - **ReserveIDs** stays the bridge atomic permanently (see the
    "ReserveIDs" note below: splitting it causes cross-process meshID
    collisions).
  - The facade mutators (`MakeEmpty`, `SetTriRefs`, `AddMeshIDTransform`,
    `SetProperties`, …) remain the working-copy commit mechanism.

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
