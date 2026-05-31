// Package manifold's Impl/MutableImpl facade types correspond to
// C++ Manifold::Impl — the single class that all Impl member methods
// live on. The split into Impl (const view) and MutableImpl (mutable)
// mirrors C++ shared_ptr<const Impl> vs shared_ptr<Impl>: there's
// only one class, but two reference flavours with different sets of
// callable methods (const vs non-const).
//
// Both are now backed by native Go storage (implStorage). MutableImpl still keeps
// a transient bridge handle for the last two C++ algorithms (Subdivide/Refine via
// runBridgeAlgo); the const Impl is pure Go. The bridge is not discarded at the end
// of the port — it is retained as the permanent C++ differential-test oracle (it
// relocates to a test-only package), so regressions, performance changes, and
// upstream merges can always be validated against the reference implementation.
package manifold

import (
	"math"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
	"github.com/firstlayer-xyz/manifold/go/internal/quickhull"
)

// meshIDCounter is the Go port of the C++ static std::atomic<uint32_t>
// Manifold::Impl::meshIDCounter_(1) (impl.cpp:91) — a single process-wide,
// monotonic source of unique mesh IDs. Initialized to 1 in init() (atomic.Uint32's
// zero value is 0, so the C++ static initializer's value of 1 is set explicitly).
var meshIDCounter atomic.Uint32

func init() { meshIDCounter.Store(1) }

// reserveIDs is the Go port of Manifold::Impl::ReserveIDs (impl.cpp:93):
// meshIDCounter_.fetch_add(n, relaxed), returning the PRE-increment value
// (atomic.Uint32.Add returns the post-increment value, so subtract n). reserveIDs(0)
// is a non-incrementing snapshot read of the counter — the direct meshIDCounter_
// reads in csg_tree.cpp:277 (CsgOpNode meshID snapshot) and boolean_result.cpp:523
// (UpdateReference offsetQ).
func reserveIDs(n uint32) uint32 { return meshIDCounter.Add(n) - n }

// Impl is a const view of a Manifold::Impl. Mirrors C++
// shared_ptr<const Manifold::Impl>: callable methods are exactly the
// const member methods on Manifold::Impl.
//
// Construct via getImpl(*Manifold) — the equivalent of the C++
// GetCsgLeafNode().GetImpl() chain. Pair with Delete (it releases
// the shared_ptr reference; the underlying Impl lives on inside the
// Manifold).
type Impl struct {
	// s is the native storage. getImpl aliases the owning Manifold's storage
	// directly (oracle-migration step 3: no cgo read — the const view reads s,
	// mirroring MutableImpl). The const Impl is read-only by contract, so the
	// alias is safe; mutating paths go through Copy, which clones s.
	s *implStorage
	// coll is the native Go collider (persistent, native-Impl-storage Phase 1).
	// Carried from the owning Manifold (set by ToManifold / refitted by Transform);
	// nil for finalized meshes that haven't needed one yet — ensureCollider lazily
	// builds it from the Morton-sorted faces on demand. Mirrors C++ Impl::collider_.
	coll *collider.Collider
}

// MutableImpl is the mutable counterpart of Impl — mirrors C++
// shared_ptr<Manifold::Impl>. Carries both the mutating method set
// and the read-only method set (Go has no const so we just expose
// both; users follow the convention by calling read methods on Impl
// when they only need const access).
//
// Construct via newImpl() (fresh empty) or via (*Impl).Copy() (deep
// copy of a const view). Pair with Delete.
type MutableImpl struct {
	h    *bridge.MutableImpl
	s    *implStorage       // native storage being migrated (see Impl.s)
	coll *collider.Collider // native collider (see Impl.coll)
}

// getImpl returns a const view of m's underlying Impl, mirroring
// C++ `m.GetCsgLeafNode().GetImpl()`. Caller must Delete the returned
// Impl (releases the shared_ptr reference held by this view). The native
// collider, if the Manifold carries one, comes along (set by Transform/ToManifold).
func getImpl(m *Manifold) *Impl {
	return &Impl{s: m.s, coll: m.coll}
}

// newImpl returns a freshly-allocated, empty MutableImpl. Mirrors
// C++ `std::make_shared<Manifold::Impl>()`.
func newImpl() *MutableImpl {
	return &MutableImpl{h: bridge.NewMutableImpl(), s: newImplStorage()}
}

// Delete releases this const view. The const Impl aliases the owning Manifold's
// native storage (getImpl), so there is nothing of its own to free; the method is
// kept because callers `defer impl.Delete()` by convention.
func (i *Impl) Delete() {}

// Delete releases this mutable Impl's transient bridge handle (the only resource
// it owns — the native storage is shared with whatever consumed it via ToManifold).
// Guarded against the nil h that ToManifold leaves behind.
func (mi *MutableImpl) Delete() {
	if mi.h != nil {
		mi.h.Delete()
		mi.h = nil
	}
}

// Copy returns a fresh mutable Impl initialized from this const view.
// Mirrors C++ `std::make_shared<Manifold::Impl>(*src)` — including a deep copy
// of collider_ (impl.cpp:675), so a refit on the copy never mutates the source.
func (i *Impl) Copy() *MutableImpl {
	// Clone the native storage; allocate a fresh transient bridge handle only so the
	// still-C++ Subdivide/Refine (runBridgeAlgo) have somewhere to marshal into.
	mi := &MutableImpl{h: bridge.NewMutableImpl(), s: i.s.clone()}
	if i.coll != nil {
		mi.coll = i.coll.Copy()
	}
	return mi
}

// ToManifold seals this MutableImpl into a published Manifold, mirroring C++
// `Manifold(std::make_shared<CsgLeafNode>(impl))`. The native storage is published
// directly (oracle-migration step 3: no bridge round-trip); the native collider
// travels with the Manifold. The transient bridge handle (used only by the still-C++
// Subdivide/Refine) is released here.
func (mi *MutableImpl) ToManifold() *Manifold {
	m := &Manifold{s: mi.s, coll: mi.coll}
	if mi.h != nil {
		mi.h.Delete()
		mi.h = nil
	}
	return m
}

// ensureCollider returns the native collider, lazily building it from the
// Morton-sorted face boxes when absent. Lazy build is only reached for finalized
// meshes (faces in Morton order, so collider.New's leaf==face holds); transformed
// meshes always carry a refitted collider via Transform, so they never rebuild.
func (i *Impl) ensureCollider() *collider.Collider {
	if i.coll == nil {
		fb, fm := i.GetFaceBoxMorton()
		i.coll = collider.New(fb, fm)
	}
	return i.coll
}

// --- Read-only methods (mirror const members of C++ Manifold::Impl)

// Verts returns the impl's vertPos_ as a read-only Go slice.
func (i *Impl) Verts() []geom.Vec3 { return i.s.vertPos }

// HalfedgeStarts returns halfedge_.start_ as a read-only slice.
func (i *Impl) HalfedgeStarts() []int32 { return i.s.halfedge.Starts() }

// HalfedgePairs returns halfedge_.paired_ as a read-only slice.
func (i *Impl) HalfedgePairs() []int32 { return i.s.halfedge.Paired() }

// HalfedgeProps returns halfedge_.propVert_ as a read-only slice.
func (i *Impl) HalfedgeProps() []int32 { return i.s.halfedge.PropVert() }

// FaceNormals returns faceNormal_ as a read-only slice. Empty when
// face normals haven't been cached.
func (i *Impl) FaceNormals() []geom.Vec3 { return i.s.faceNormal }

// VertNormals returns vertNormal_ as a read-only slice. Empty when
// vert normals haven't been cached.
func (i *Impl) VertNormals() []geom.Vec3 { return i.s.vertNormal }

// HalfedgeTangents returns halfedgeTangent_ as a flat 4-per-element slice
// (a fresh copy of the native Vec4 storage).
func (i *Impl) HalfedgeTangents() []float64 { return tangentsToFlat(i.s.halfedgeTangent) }

// Properties returns properties_ as a read-only slice.
func (i *Impl) Properties() []float64 { return i.s.properties }

// meshIDRelation is the manifold-package value type for one flattened
// meshRelation_.meshIDtransform entry (the map key meshID + its Relation). It
// replaces the bridge.MeshIDRelation leak in the accessor API.
type meshIDRelation struct {
	MeshID, OriginalID int
	Transform          geom.Mat3x4
	BackSide           bool
	HasNormals         bool
}

// TriRefs returns the meshRelation_.triRef array (the native backing slice).
func (i *Impl) TriRefs() []mesh.TriRef { return i.s.meshRelation.TriRef }

// MeshIDTransforms returns the meshRelation_.meshIDtransform map's
// entries in std::map order (ascending meshID).
func (i *Impl) MeshIDTransforms() []meshIDRelation {
	return meshIDTransformsNative(&i.s.meshRelation)
}

// BBox returns bBox_'s min/max corners.
func (i *Impl) BBox() (min, max geom.Vec3) { return i.s.bBox.Min, i.s.bBox.Max }

// HalfedgeCount returns halfedge_.size().
func (i *Impl) HalfedgeCount() int { return i.s.halfedge.Size() }

// NumTri returns NumTri() = halfedge_.size() / 3.
func (i *Impl) NumTri() int { return i.s.halfedge.Size() / 3 }

// NumVert returns NumVert() = vertPos_.size().
func (i *Impl) NumVert() int { return len(i.s.vertPos) }

// implScalars is the manifold-package value type for Manifold::Impl's small
// scalar fields. It replaces the bridge.ImplScalars leak in the accessor API
// (oracle-migration step 1: decouple the public accessor types from the bridge).
type implScalars struct {
	NumProp             int
	PropertiesSize      int
	HalfedgeTangentSize int
	Tolerance           float64
	Epsilon             float64
	OriginalID          int
	Status              int
}

// Scalars returns the small scalar fields (NumProp, Tolerance,
// Epsilon, OriginalID, Status, PropertiesSize, HalfedgeTangentSize).
func (i *Impl) Scalars() implScalars {
	s := i.s
	return implScalars{
		NumProp:             s.numProp,
		PropertiesSize:      len(s.properties),
		HalfedgeTangentSize: len(s.halfedgeTangent),
		Tolerance:           s.tolerance,
		Epsilon:             s.epsilon,
		OriginalID:          s.meshRelation.OriginalID,
		Status:              int(s.status),
	}
}

// AllHaveNormals is the Go port of Impl::AllHaveNormals (src/impl.h):
// true iff every entry of meshRelation_.meshIDtransform has hasNormals
// set. Returns false on an empty map.
func (i *Impl) AllHaveNormals() bool {
	rels := i.MeshIDTransforms()
	if len(rels) == 0 {
		return false
	}
	for _, r := range rels {
		if !r.HasNormals {
			return false
		}
	}
	return true
}

// IsFinite is the Go port of C++ Manifold::Impl::IsFinite
// (src/properties.cpp): true iff every vertex position has finite
// (non-NaN, non-inf) components. Uses parallel.TransformReduce with
// && reduction, matching the C++ transform_reduce.
//
// Note: this is a stronger check than `bBox_.IsFinite()` — bBox is
// computed by CalculateBBox while skipping NaNs, so a partial-NaN
// vert set can still yield a finite bBox.
func (i *Impl) IsFinite() bool {
	return parallel.TransformReduce(
		parallel.AutoPolicy(i.NumVert(), 100000),
		i.Verts(), true,
		func(a, b bool) bool { return a && b },
		func(v geom.Vec3) bool { return v.IsFinite() },
	)
}

// GetMeshGL returns the float32/uint32 mesh export.
func (i *Impl) GetMeshGL(normalIdx int) meshGLP[float32, uint32] {
	return getMeshGLImpl[float32, uint32](i, normalIdx)
}

// GetMeshGL64 returns the float64/uint64 mesh export.
func (i *Impl) GetMeshGL64(normalIdx int) meshGLP[float64, uint64] {
	return getMeshGLImpl[float64, uint64](i, normalIdx)
}

// RayCast is the Go port of Impl::RayCast — see internal/boolean.RayCast (the
// Boolean3 kernel cascade). Builds an ephemeral face collider, then casts the
// ray through the native kernel. Differential-tested vs the bridge in
// TestRayCast_VsCpp.
func (i *Impl) RayCast(origin, endpoint geom.Vec3) []RayHit {
	if i.NumTri() == 0 {
		return nil
	}
	col := i.ensureCollider()
	hits := boolean.RayCast(i.Verts(), i.VertNormals(), i.FaceNormals(),
		i.HalfedgeStarts(), i.HalfedgePairs(), col, origin, endpoint)
	out := make([]RayHit, len(hits))
	for j, h := range hits {
		out[j] = RayHit{FaceID: h.FaceID, Distance: h.Distance, Position: h.Position, Normal: h.Normal}
	}
	return out
}

// Minkowski is native — see impl_minkowski.go.

// --- MutableImpl: read accessors (mirror non-const member methods
// that also read state; Go has no const so we just expose both)

// Verts returns vertPos_ as a writable Go slice. Mutations via the
// slice are visible to the underlying storage.
func (mi *MutableImpl) Verts() []geom.Vec3 { return mi.s.vertPos }

// VertNormals returns vertNormal_ as a writable Go slice.
func (mi *MutableImpl) VertNormals() []geom.Vec3 { return mi.s.vertNormal }

// FaceNormals returns faceNormal_ as a writable Go slice.
func (mi *MutableImpl) FaceNormals() []geom.Vec3 { return mi.s.faceNormal }

// HalfedgeStarts is the halfedge_.start_ column (read-write alias).
func (mi *MutableImpl) HalfedgeStarts() []int32 { return mi.s.halfedge.Starts() }

// HalfedgePairs is the halfedge_.paired_ column.
func (mi *MutableImpl) HalfedgePairs() []int32 { return mi.s.halfedge.Paired() }

// HalfedgeProps is the halfedge_.propVert_ column.
func (mi *MutableImpl) HalfedgeProps() []int32 { return mi.s.halfedge.PropVert() }

// HalfedgeTangents returns halfedgeTangent_ as a flat 4-per-element slice
// (a fresh copy of the native Vec4 storage).
func (mi *MutableImpl) HalfedgeTangents() []float64 { return tangentsToFlat(mi.s.halfedgeTangent) }

// Properties is the read-write properties_ slice.
func (mi *MutableImpl) Properties() []float64 { return mi.s.properties }

// TriRefs returns the meshRelation_.triRef array (the native backing slice).
func (mi *MutableImpl) TriRefs() []mesh.TriRef { return mi.s.meshRelation.TriRef }

// MeshIDTransforms returns the meshIDtransform map entries in ascending
// meshID order (mirroring std::map iteration).
func (mi *MutableImpl) MeshIDTransforms() []meshIDRelation {
	return meshIDTransformsNative(&mi.s.meshRelation)
}

// BBox returns bBox_'s corners.
func (mi *MutableImpl) BBox() (min, max geom.Vec3) { return mi.s.bBox.Min, mi.s.bBox.Max }

// HalfedgeCount returns halfedge_.size().
func (mi *MutableImpl) HalfedgeCount() int { return mi.s.halfedge.Size() }

// NumTri returns NumTri().
func (mi *MutableImpl) NumTri() int { return mi.HalfedgeCount() / 3 }

// NumVert returns NumVert() = vertPos_.size().
func (mi *MutableImpl) NumVert() int { return len(mi.s.vertPos) }

// NumProp returns numProp_.
func (mi *MutableImpl) NumProp() int { return mi.s.numProp }

// Tolerance returns tolerance_.
func (mi *MutableImpl) Tolerance() float64 { return mi.s.tolerance }

// AllHaveNormals mirrors Impl::AllHaveNormals on a mutable Impl:
// true iff every entry of meshRelation_.meshIDtransform has
// hasNormals set. Returns false on an empty map.
func (mi *MutableImpl) AllHaveNormals() bool {
	rels := mi.MeshIDTransforms()
	if len(rels) == 0 {
		return false
	}
	for _, r := range rels {
		if !r.HasNormals {
			return false
		}
	}
	return true
}

// IsFinite mirrors Impl::IsFinite on a mutable Impl: true iff every
// vertex position has finite components. See Impl.IsFinite for the
// algorithm (parallel transform_reduce over vertPos_).
func (mi *MutableImpl) IsFinite() bool {
	return parallel.TransformReduce(
		parallel.AutoPolicy(mi.NumVert(), 100000),
		mi.Verts(), true,
		func(a, b bool) bool { return a && b },
		func(v geom.Vec3) bool { return v.IsFinite() },
	)
}

// SetToleranceValue assigns tolerance_ directly.
func (mi *MutableImpl) SetToleranceValue(tol float64) { mi.s.tolerance = tol }

// SetEpsilonValue assigns epsilon_ directly.
func (mi *MutableImpl) SetEpsilonValue(eps float64) { mi.s.epsilon = eps }

// MakeEmpty is the Go port of Manifold::Impl::MakeEmpty (src/impl.cpp:577):
// clear the geometry/relation/collider and set status_. Matches C++ exactly —
// properties_, numProp_, epsilon_ and tolerance_ are NOT cleared.
func (mi *MutableImpl) MakeEmpty(status int) {
	mi.s.bBox = geom.EmptyBox()
	mi.s.vertPos = nil
	mi.s.halfedge = mesh.NewHalfedges(nil, nil, nil)
	mi.s.vertNormal = nil
	mi.s.faceNormal = nil
	mi.s.halfedgeTangent = nil
	mi.s.meshRelation = mesh.NewMeshRelationD()
	mi.coll = nil
	mi.s.status = Error(status)
}

// ResizeVerts resizes vertPos_ to n elements.
func (mi *MutableImpl) ResizeVerts(n int) { mi.s.vertPos = resizeVec3(mi.s.vertPos, n) }

// ResizeVertNormals resizes vertNormal_ to n elements.
func (mi *MutableImpl) ResizeVertNormals(n int) { mi.s.vertNormal = resizeVec3(mi.s.vertNormal, n) }

// ResizeFaceNormals resizes faceNormal_ to n elements.
func (mi *MutableImpl) ResizeFaceNormals(n int) { mi.s.faceNormal = resizeVec3(mi.s.faceNormal, n) }

// Epsilon returns epsilon_.
func (mi *MutableImpl) Epsilon() float64 { return mi.s.epsilon }

// SetHalfedgesRaw bulk-assigns halfedge_ from parallel start/prop/paired arrays.
// (Bridge arg order is starts, props, paireds — native column order is
// starts, paired, propVert.)
func (mi *MutableImpl) SetHalfedgesRaw(starts, props, paireds []int32) {
	mi.s.halfedge = mesh.NewHalfedges(starts, paireds, props)
}

// SetProperties bulk-assigns properties_.
func (mi *MutableImpl) SetProperties(data []float64) { mi.s.properties = data }

// SetNumProp assigns numProp_.
func (mi *MutableImpl) SetNumProp(n int) { mi.s.numProp = n }

// SetHalfedgeTangents bulk-assigns halfedgeTangent_ (flat 4-per-element).
func (mi *MutableImpl) SetHalfedgeTangents(data []float64) {
	mi.s.halfedgeTangent = tangentsFromFlat(data)
}

// SetTriRefs bulk-assigns meshRelation_.triRef from parallel id arrays.
func (mi *MutableImpl) SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs []int32) {
	refs := make([]mesh.TriRef, len(meshIDs))
	for i := range meshIDs {
		refs[i] = mesh.TriRef{
			MeshID: int(meshIDs[i]), OriginalID: int(originalIDs[i]),
			FaceID: int(faceIDs[i]), CoplanarID: int(coplanarIDs[i]),
		}
	}
	mi.s.meshRelation.TriRef = refs
}

// SetCoplanarIDs rewrites only the coplanarID column of meshRelation_.triRef.
func (mi *MutableImpl) SetCoplanarIDs(ids []int32) {
	for i := range ids {
		mi.s.meshRelation.TriRef[i].CoplanarID = int(ids[i])
	}
}

// SetBBox assigns bBox_.
func (mi *MutableImpl) SetBBox(min, max geom.Vec3) { mi.s.bBox = geom.Box{Min: min, Max: max} }

// SetMeshRelationOriginalID assigns meshRelation_.originalID.
func (mi *MutableImpl) SetMeshRelationOriginalID(id int) { mi.s.meshRelation.OriginalID = id }

// ClearMeshIDTransforms empties meshRelation_.meshIDtransform.
func (mi *MutableImpl) ClearMeshIDTransforms() { mi.s.meshRelation.MeshIDTransform.Clear() }

// AddMeshIDTransform inserts one meshRelation_.meshIDtransform entry.
func (mi *MutableImpl) AddMeshIDTransform(meshID, originalID int, transform geom.Mat3x4, backSide, hasNormals bool) {
	mi.s.meshRelation.MeshIDTransform.Set(meshID, mesh.Relation{
		OriginalID: originalID, Transform: transform,
		BackSide: backSide, HasNormals: hasNormals,
	})
}

// --- Bridge-only mutators (still C++ algorithms) ---
//
// Subdivide / Refine are the last storage-mutating algorithms still running in
// C++. Each is bracketed by marshalRoundTrip: push the native storage into the
// bridge handle, run the C++ pass, then re-read the mutated storage back. The
// native side stays the source of truth so the surrounding native code (e.g.
// Sphere reading Verts() after SubdivideN, ToManifold) sees the result.
//
// SimplifyTopology / RemoveDegenerates / CreateTangents are native Go — see
// impl_simplify.go / impl_smoothing_tangents.go.

// Subdivide calls C++ Impl::Subdivide with the constant-n splits lambda.
func (mi *MutableImpl) SubdivideN(n int) {
	// Native topological subdivision (no smoothing): Subdivide([n]{n-1}, false).
	// Subdivide rebuilds the halfedges but does not finalize; the geometry changed,
	// so any cached collider is stale.
	mi.Subdivide(func(geom.Vec3, geom.Vec4, geom.Vec4) int { return n - 1 }, false)
	mi.coll = nil
}

// RefineN is the native Refine with constant n-1 splits (Manifold::Refine).
func (mi *MutableImpl) RefineN(n int) {
	mi.Refine(func(geom.Vec3, geom.Vec4, geom.Vec4) int { return n - 1 }, false)
}

// RefineToLength is the native Refine splitting each edge into |edge|/length
// pieces (Manifold::RefineToLength).
func (mi *MutableImpl) RefineToLength(length float64) {
	mi.Refine(func(edge geom.Vec3, _, _ geom.Vec4) int {
		return int(edge.Length() / length)
	}, false)
}

// RefineToTolerance is the native Refine splitting each edge by a tolerance-driven
// arc heuristic (Manifold::RefineToTolerance); keepInterior thickens strip-like
// triangulations.
func (mi *MutableImpl) RefineToTolerance(tol float64) {
	mi.Refine(func(edge geom.Vec3, tangentStart, tangentEnd geom.Vec4) int {
		edgeNorm := edge.Normalize()
		tStart := tangentStart.Vec3()
		tEnd := tangentEnd.Vec3()
		start := tStart.Sub(edgeNorm.Scale(edgeNorm.Dot(tStart)))
		end := tEnd.Sub(edgeNorm.Scale(edgeNorm.Dot(tEnd)))
		d := 0.5*(start.Length()+end.Length()) + start.Sub(end).Length()
		return int(math.Sqrt(3 * d / (4 * tol)))
	}, true)
}

// runBridgeAlgo runs a still-C++ storage-mutating algorithm against the bridge
// handle while keeping the native implStorage as the source of truth: marshal
// s -> h, run the C++ pass (which mutates h), then reload h -> s.
func (mi *MutableImpl) runBridgeAlgo(algo func()) {
	marshalImplStorageToBridge(mi.s, mi.h)
	algo()
	mi.reloadFromBridge()
	// Refine/Subdivide change the geometry, so any collider cloned from the
	// source (Copy) is now stale. C++ rebuilds collider_ inside Refine; the Go
	// side invalidates it here so ensureCollider rebuilds from the new geometry.
	mi.coll = nil
}

// Hull is the Go port of C++ Manifold::Impl::Hull (src/quickhull.cpp:834).
// Runs QuickHull3D on the input vertex cloud, writes the resulting
// convex-hull halfedges + vertices into this Impl, then runs the
// standard finalize tail (CalculateBBox → SetEpsilon →
// InitializeOriginal → SortGeometry → SetNormalsAndCoplanar).
func (mi *MutableImpl) Hull(vertPos []geom.Vec3) {
	if len(vertPos) == 0 {
		return
	}
	qh := quickhull.NewQuickHull(vertPos)
	hes, verts := qh.BuildMesh(quickhull.DefaultEpsilon)

	// Materialize verts into the impl storage.
	mi.ResizeVerts(len(verts))
	copy(mi.Verts(), verts)

	// Translate quickhull.Halfedge → impl halfedge_ arrays.
	// QuickHull emits face-major triples; propVert = startVert for
	// the single-arg form (no extra props).
	n := len(hes)
	starts := make([]int32, n)
	props := make([]int32, n)
	paireds := make([]int32, n)
	for i := 0; i < n; i++ {
		starts[i] = int32(hes[i].StartVert)
		props[i] = int32(hes[i].StartVert)
		paireds[i] = int32(hes[i].PairedHalfedge)
	}
	mi.SetHalfedgesRaw(starts, props, paireds)

	mi.CalculateBBox()
	mi.SetEpsilon(-1, false)
	mi.InitializeOriginal()
	mi.SortGeometry()
	mi.SetNormalsAndCoplanar()
}
