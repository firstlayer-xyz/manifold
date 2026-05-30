// Package manifold's Impl/MutableImpl facade types correspond to
// C++ Manifold::Impl — the single class that all Impl member methods
// live on. The split into Impl (const view) and MutableImpl (mutable)
// mirrors C++ shared_ptr<const Impl> vs shared_ptr<Impl>: there's
// only one class, but two reference flavours with different sets of
// callable methods (const vs non-const).
//
// During the port, both wrap bridge handles that hold the underlying
// C++ Impl. Once everything is ported, Impl/MutableImpl become plain
// Go structs and the bridge package goes away.
package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
	"github.com/firstlayer-xyz/manifold/go/internal/quickhull"
)

// Impl is a const view of a Manifold::Impl. Mirrors C++
// shared_ptr<const Manifold::Impl>: callable methods are exactly the
// const member methods on Manifold::Impl.
//
// Construct via getImpl(*Manifold) — the equivalent of the C++
// GetCsgLeafNode().GetImpl() chain. Pair with Delete (it releases
// the shared_ptr reference; the underlying Impl lives on inside the
// Manifold).
type Impl struct {
	h *bridge.Impl
	// s holds the native Go storage being migrated off the bridge (native-Impl-
	// storage Phase 2b). Dormant in Step A (accessors still forward to h); Step B
	// flips the accessor bodies to read/write s, with getImpl marshalling in and
	// ToManifold marshalling out.
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
	return &Impl{h: bridge.GetImpl(m.h), s: newImplStorage(), coll: m.coll}
}

// newImpl returns a freshly-allocated, empty MutableImpl. Mirrors
// C++ `std::make_shared<Manifold::Impl>()`.
func newImpl() *MutableImpl {
	return &MutableImpl{h: bridge.NewMutableImpl(), s: newImplStorage()}
}

// Delete releases this const view's shared_ptr reference.
func (i *Impl) Delete() { i.h.Delete() }

// Delete releases this mutable Impl's shared_ptr reference.
func (mi *MutableImpl) Delete() { mi.h.Delete() }

// Copy returns a fresh mutable Impl initialized from this const view.
// Mirrors C++ `std::make_shared<Manifold::Impl>(*src)` — including a deep copy
// of collider_ (impl.cpp:675), so a refit on the copy never mutates the source.
func (i *Impl) Copy() *MutableImpl {
	mi := &MutableImpl{h: i.h.Copy(), s: newImplStorage()}
	if i.coll != nil {
		mi.coll = i.coll.Copy()
	}
	return mi
}

// ToManifold seals this MutableImpl into a published Manifold,
// mirroring C++ `Manifold(std::make_shared<CsgLeafNode>(impl))`. The native
// collider travels with the published Manifold so a later getImpl recovers it.
func (mi *MutableImpl) ToManifold() *Manifold {
	m := wrap(mi.h.ToManifold())
	m.coll = mi.coll
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

// Verts returns the impl's vertPos_ as a read-only Go slice aliasing
// C++ memory. Mirrors C++ `impl->vertPos_` access.
func (i *Impl) Verts() []geom.Vec3 { return i.h.Verts() }

// HalfedgeStarts returns halfedge_.start_ as a read-only slice.
func (i *Impl) HalfedgeStarts() []int32 { return i.h.HalfedgeStarts() }

// HalfedgePairs returns halfedge_.paired_ as a read-only slice.
func (i *Impl) HalfedgePairs() []int32 { return i.h.HalfedgePairs() }

// HalfedgeProps returns halfedge_.propVert_ as a read-only slice.
func (i *Impl) HalfedgeProps() []int32 { return i.h.HalfedgeProps() }

// FaceNormals returns faceNormal_ as a read-only slice. Empty when
// face normals haven't been cached.
func (i *Impl) FaceNormals() []geom.Vec3 { return i.h.FaceNormals() }

// VertNormals returns vertNormal_ as a read-only slice. Empty when
// vert normals haven't been cached.
func (i *Impl) VertNormals() []geom.Vec3 { return i.h.VertNormals() }

// HalfedgeTangents returns halfedgeTangent_ as a read-only slice
// (4 doubles per element, packed vec4).
func (i *Impl) HalfedgeTangents() []float64 { return i.h.HalfedgeTangents() }

// Properties returns properties_ as a read-only slice.
func (i *Impl) Properties() []float64 { return i.h.Properties() }

// TriRefs returns the meshRelation_.triRef array as a copied Go
// slice (each TriRef carries meshID/originalID/faceID/coplanarID).
func (i *Impl) TriRefs() []bridge.TriRef { return i.h.TriRefs() }

// MeshIDTransforms returns the meshRelation_.meshIDtransform map's
// entries in std::map order (ascending meshID).
func (i *Impl) MeshIDTransforms() []bridge.MeshIDRelation { return i.h.MeshIDTransforms() }

// BBox returns bBox_'s min/max corners.
func (i *Impl) BBox() (min, max geom.Vec3) { return i.h.BBox() }

// HalfedgeCount returns halfedge_.size().
func (i *Impl) HalfedgeCount() int { return i.h.HalfedgeCount() }

// NumTri returns NumTri() = halfedge_.size() / 3.
func (i *Impl) NumTri() int { return i.h.HalfedgeCount() / 3 }

// NumVert returns NumVert() = vertPos_.size().
func (i *Impl) NumVert() int { return len(i.h.Verts()) }

// Scalars returns the small scalar fields (NumProp, Tolerance,
// Epsilon, OriginalID, Status, PropertiesSize, HalfedgeTangentSize).
func (i *Impl) Scalars() bridge.ImplScalars { return i.h.Scalars() }

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
func (i *Impl) RayCast(origin, endpoint geom.Vec3) []bridge.RayHit {
	if i.NumTri() == 0 {
		return nil
	}
	col := i.ensureCollider()
	hits := boolean.RayCast(i.Verts(), i.VertNormals(), i.FaceNormals(),
		i.HalfedgeStarts(), i.HalfedgePairs(), col, origin, endpoint)
	out := make([]bridge.RayHit, len(hits))
	for j, h := range hits {
		out[j] = bridge.RayHit{FaceID: h.FaceID, Distance: h.Distance, Position: h.Position, Normal: h.Normal}
	}
	return out
}

// Minkowski wraps Impl::Minkowski (still in C++).
func (i *Impl) Minkowski(other *Impl, inset bool) *Manifold {
	return wrap(i.h.Minkowski(other.h, inset))
}

// newBoolean3 wraps the C++ Boolean3(a, b, op) ctor for Split etc.
func newBoolean3(a, b *Impl, op int) *bridge.Boolean3 {
	return bridge.NewBoolean3(a.h, b.h, op)
}

// --- MutableImpl: read accessors (mirror non-const member methods
// that also read state; Go has no const so we just expose both)

// Verts returns vertPos_ as a writable Go slice aliasing C++ memory.
// Mutations via the slice are visible to the underlying Impl.
func (mi *MutableImpl) Verts() []geom.Vec3 { return mi.h.Verts() }

// VertNormals returns vertNormal_ as a writable Go slice.
func (mi *MutableImpl) VertNormals() []geom.Vec3 { return mi.h.VertNormals() }

// FaceNormals returns faceNormal_ as a writable Go slice.
func (mi *MutableImpl) FaceNormals() []geom.Vec3 { return mi.h.FaceNormalsMut() }

// HalfedgeStarts is the read-only halfedge_.start_ view on a
// mutable Impl. Use SetHalfedges (or a Go-side rebuild) to mutate.
func (mi *MutableImpl) HalfedgeStarts() []int32 { return mi.h.HalfedgeStartsRO() }

// HalfedgePairs is the read-only halfedge_.paired_ view.
func (mi *MutableImpl) HalfedgePairs() []int32 { return mi.h.HalfedgePairsRO() }

// HalfedgeProps is the read-only halfedge_.propVert_ view.
func (mi *MutableImpl) HalfedgeProps() []int32 { return mi.h.HalfedgePropsRO() }

// HalfedgeTangents is the read-only halfedgeTangent_ view (4 doubles
// per element).
func (mi *MutableImpl) HalfedgeTangents() []float64 { return mi.h.HalfedgeTangents() }

// Properties is the read-only properties_ view.
func (mi *MutableImpl) Properties() []float64 { return mi.h.Properties() }

// TriRefs returns the meshRelation_.triRef array as a copied Go slice.
func (mi *MutableImpl) TriRefs() []bridge.TriRef { return mi.h.TriRefs() }

// MeshIDTransforms returns the meshIDtransform map entries.
func (mi *MutableImpl) MeshIDTransforms() []bridge.MeshIDRelation {
	return mi.h.MeshIDTransforms()
}

// BBox returns bBox_'s corners.
func (mi *MutableImpl) BBox() (min, max geom.Vec3) { return mi.h.GetBBox() }

// HalfedgeCount returns halfedge_.size().
func (mi *MutableImpl) HalfedgeCount() int { return len(mi.h.HalfedgeStartsRO()) }

// NumTri returns NumTri().
func (mi *MutableImpl) NumTri() int { return mi.HalfedgeCount() / 3 }

// NumVert returns NumVert() = vertPos_.size().
func (mi *MutableImpl) NumVert() int { return len(mi.h.Verts()) }

// NumProp returns numProp_.
func (mi *MutableImpl) NumProp() int { return mi.h.NumProp() }

// Tolerance returns tolerance_.
func (mi *MutableImpl) Tolerance() float64 { return mi.h.GetTolerance() }

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
func (mi *MutableImpl) SetToleranceValue(tol float64) { mi.h.SetToleranceValue(tol) }

// SetEpsilonValue assigns epsilon_ directly.
func (mi *MutableImpl) SetEpsilonValue(eps float64) { mi.h.SetEpsilonValue(eps) }

// MakeEmpty wraps Impl::MakeEmpty(Error).
func (mi *MutableImpl) MakeEmpty(status int) { mi.h.MakeEmpty(status) }

// ResizeVerts resizes vertPos_ to n elements.
func (mi *MutableImpl) ResizeVerts(n int) { mi.h.ResizeVerts(n) }

// ResizeVertNormals resizes vertNormal_ to n elements.
func (mi *MutableImpl) ResizeVertNormals(n int) { mi.h.ResizeVertNormals(n) }

// ResizeFaceNormals resizes faceNormal_ to n elements.
func (mi *MutableImpl) ResizeFaceNormals(n int) { mi.h.ResizeFaceNormals(n) }

// Epsilon returns epsilon_.
func (mi *MutableImpl) Epsilon() float64 { return mi.h.GetEpsilon() }

// SetHalfedgesRaw bulk-assigns halfedge_ from parallel start/prop/paired arrays.
func (mi *MutableImpl) SetHalfedgesRaw(starts, props, paireds []int32) {
	mi.h.SetHalfedgesRaw(starts, props, paireds)
}

// SetProperties bulk-assigns properties_.
func (mi *MutableImpl) SetProperties(data []float64) { mi.h.SetProperties(data) }

// SetNumProp assigns numProp_.
func (mi *MutableImpl) SetNumProp(n int) { mi.h.SetNumProp(n) }

// SetHalfedgeTangents bulk-assigns halfedgeTangent_ (flat 4-per-element).
func (mi *MutableImpl) SetHalfedgeTangents(data []float64) { mi.h.SetHalfedgeTangents(data) }

// SetTriRefs bulk-assigns meshRelation_.triRef from parallel id arrays.
func (mi *MutableImpl) SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs []int32) {
	mi.h.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
}

// SetCoplanarIDs rewrites only the coplanarID column of meshRelation_.triRef.
func (mi *MutableImpl) SetCoplanarIDs(ids []int32) { mi.h.SetCoplanarIDs(ids) }

// SetBBox assigns bBox_.
func (mi *MutableImpl) SetBBox(min, max geom.Vec3) { mi.h.SetBBox(min, max) }

// SetMeshRelationOriginalID assigns meshRelation_.originalID.
func (mi *MutableImpl) SetMeshRelationOriginalID(id int) { mi.h.SetMeshRelationOriginalID(id) }

// ClearMeshIDTransforms empties meshRelation_.meshIDtransform.
func (mi *MutableImpl) ClearMeshIDTransforms() { mi.h.ClearMeshIDTransforms() }

// AddMeshIDTransform inserts one meshRelation_.meshIDtransform entry.
func (mi *MutableImpl) AddMeshIDTransform(meshID, originalID int, transform [4][3]float64, backSide, hasNormals bool) {
	mi.h.AddMeshIDTransform(meshID, originalID, transform, backSide, hasNormals)
}

// --- Bridge-only mutators (still C++ algorithms) ---

// SimplifyTopology is native Go — see impl_simplify.go.

// Subdivide calls C++ Impl::Subdivide with the constant-n splits lambda.
func (mi *MutableImpl) SubdivideN(n int) { mi.h.SubdivideN(n) }

// RefineN calls C++ Impl::Refine with constant n-1 splits.
func (mi *MutableImpl) RefineN(n int) { mi.h.RefineN(n) }

// RefineToLength calls C++ Impl::Refine with edge-length-based splits.
func (mi *MutableImpl) RefineToLength(length float64) { mi.h.RefineToLength(length) }

// RefineToTolerance calls C++ Impl::Refine with tolerance-based splits.
func (mi *MutableImpl) RefineToTolerance(tol float64) { mi.h.RefineToTolerance(tol) }

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

	// Materialize verts into the bridge Impl.
	mi.h.ResizeVerts(len(verts))
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
	mi.h.SetHalfedgesRaw(starts, props, paireds)

	mi.CalculateBBox()
	mi.SetEpsilon(-1, false)
	mi.InitializeOriginal()
	mi.SortGeometry()
	mi.SetNormalsAndCoplanar()
}
