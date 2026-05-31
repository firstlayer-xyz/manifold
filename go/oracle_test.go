package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
)

// This file is the test-only seam between the native Go Manifold (which holds
// native storage, post oracle-migration step 3) and the C++ reference oracle
// (the bridge/reference packages, which speak in cgo handle.Manifold). Production
// no longer stores a bridge handle on a Manifold; tests that compare against the
// C++ oracle marshal across this seam.

// wrap takes ownership of a freshly-created bridge handle, marshals its Impl into
// native storage, and releases the handle. Used by tests that build a Manifold via
// a bridge constructor (e.g. cppref.ManifoldFromMeshGL64) and own the result.
func wrap(h *handle.Manifold) *Manifold {
	bi := cppref.GetImpl(h)
	m := &Manifold{s: marshalImplStorageFromBridge(bi)}
	bi.Delete()
	cppref.DeleteManifold(h)
	return m
}

// fromRefHandle marshals a reference/bridge handle's Impl into a native Manifold
// WITHOUT taking ownership of the handle — the caller retains it (and any later
// cppref.DeleteManifold). Replaces the old `&Manifold{h: refHandle}` pattern.
func fromRefHandle(h *handle.Manifold) *Manifold {
	bi := cppref.GetImpl(h)
	defer bi.Delete()
	return &Manifold{s: marshalImplStorageFromBridge(bi)}
}

// refHandle turns this Manifold's native storage into a fresh, fully-valid C++
// handle for the reference oracle. It round-trips through the MeshGL ingest path:
// the C++ Manifold(MeshGL64) constructor re-finalizes (Morton-sorts the faces and
// builds collider_ from the sorted leaves), so the handle is valid for ANY native
// mesh — including transformed meshes whose faces are no longer Morton-sorted, on
// which a direct Collider rebuild (which assumes sorted input) would corrupt or
// crash. Replaces the old `m.h` field access. Each call allocates a new handle the
// caller is responsible for (cppref.DeleteManifold).
func (m *Manifold) refHandle() *handle.Manifold {
	gl := m.GetMeshGL64(-1)
	return cppref.ManifoldFromMeshGL64(
		gl.NumProp,
		gl.VertProperties, gl.TriVerts,
		gl.MergeFromVert, gl.MergeToVert,
		gl.RunIndex, gl.RunOriginalID, gl.RunTransform, gl.RunFlags,
		gl.FaceID, gl.HalfedgeTangent,
		gl.Tolerance,
	)
}

// --- bridge marshalling seam (test-only) ---
//
// These move the native implStorage across the cgo boundary for the differential
// oracle. They live in a _test.go file (package manifold) because they need access
// to the unexported implStorage AND the test-only bridge package; production no
// longer marshals storage to/from C++.

// marshalImplStorageFromBridge reads a C++ Impl (via the bridge accessors) into a
// freshly-owned native implStorage. Slices are copied so the storage is independent
// of the bridge handle's lifetime.
func marshalImplStorageFromBridge(h *cppref.Impl) *implStorage {
	s := newImplStorage()
	scal := h.Scalars()
	s.numProp = scal.NumProp
	s.epsilon = scal.Epsilon
	s.tolerance = scal.Tolerance
	s.status = Error(scal.Status)

	s.vertPos = append([]geom.Vec3(nil), h.Verts()...)
	s.vertNormal = append([]geom.Vec3(nil), h.VertNormals()...)
	s.faceNormal = append([]geom.Vec3(nil), h.FaceNormals()...)
	s.properties = append([]float64(nil), h.Properties()...)
	s.halfedge = mesh.NewHalfedges(
		append([]int32(nil), h.HalfedgeStarts()...),
		append([]int32(nil), h.HalfedgePairs()...),
		append([]int32(nil), h.HalfedgeProps()...),
	)
	s.halfedgeTangent = tangentsFromFlat(h.HalfedgeTangents())

	min, max := h.BBox()
	s.bBox = geom.Box{Min: min, Max: max}

	s.meshRelation.OriginalID = scal.OriginalID
	for _, r := range h.TriRefs() {
		s.meshRelation.TriRef = append(s.meshRelation.TriRef,
			mesh.TriRef{MeshID: int(r.MeshID), OriginalID: int(r.OriginalID), FaceID: int(r.FaceID), CoplanarID: int(r.CoplanarID)})
	}
	for _, r := range h.MeshIDTransforms() { // returned in ascending-key (std::map) order
		s.meshRelation.MeshIDTransform.Set(int(r.MeshID), mesh.Relation{
			OriginalID: int(r.OriginalID), Transform: geom.Mat3x4(r.Transform),
			BackSide: r.BackSide, HasNormals: r.HasNormals,
		})
	}
	return s
}

// marshalImplStorageToBridge writes a native implStorage into a (fresh) bridge
// MutableImpl via the bulk setters — a field copy, NOT a MeshGL round-trip, so it
// preserves the no-re-finalize contract. An errored storage short-circuits via
// MakeEmpty (the only bridge path that sets a non-NoError status).
func marshalImplStorageToBridge(s *implStorage, mi *cppref.MutableImpl) {
	if s.status != NoError {
		mi.MakeEmpty(int(s.status))
		return
	}
	mi.ResizeVerts(len(s.vertPos))
	copy(mi.Verts(), s.vertPos)
	mi.ResizeVertNormals(len(s.vertNormal))
	copy(mi.VertNormals(), s.vertNormal)
	mi.ResizeFaceNormals(len(s.faceNormal))
	copy(mi.FaceNormalsMut(), s.faceNormal)

	starts, paired, propVert := s.halfedge.Raw()
	mi.SetHalfedgesRaw(starts, propVert, paired)
	mi.SetNumProp(s.numProp)
	mi.SetProperties(s.properties)
	mi.SetHalfedgeTangents(tangentsToFlat(s.halfedgeTangent))

	mi.SetBBox(s.bBox.Min, s.bBox.Max)
	mi.SetEpsilonValue(s.epsilon)
	mi.SetToleranceValue(s.tolerance)

	n := len(s.meshRelation.TriRef)
	meshIDs := make([]int32, n)
	originalIDs := make([]int32, n)
	faceIDs := make([]int32, n)
	coplanarIDs := make([]int32, n)
	for i, r := range s.meshRelation.TriRef {
		meshIDs[i], originalIDs[i], faceIDs[i], coplanarIDs[i] = int32(r.MeshID), int32(r.OriginalID), int32(r.FaceID), int32(r.CoplanarID)
	}
	mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	mi.SetMeshRelationOriginalID(s.meshRelation.OriginalID)
	mi.ClearMeshIDTransforms()
	for k, r := range s.meshRelation.MeshIDTransform.All() {
		mi.AddMeshIDTransform(k, r.OriginalID, [4][3]float64(r.Transform), r.BackSide, r.HasNormals)
	}
}

// reloadFromBridge re-reads the storage a C++ algorithm just mutated on the bridge
// handle back into the native implStorage. originalID and status are preserved from
// mi.s — the bridge MutableImpl exposes no getters for them, and the C++ ops the
// test harness runs (Subdivide/Refine/Simplify) never change identity or error state.
func reloadFromBridge(mi *MutableImpl, h *cppref.MutableImpl) {
	keepOriginalID := mi.s.meshRelation.OriginalID
	keepStatus := mi.s.status

	mi.s.vertPos = append([]geom.Vec3(nil), h.Verts()...)
	mi.s.vertNormal = append([]geom.Vec3(nil), h.VertNormals()...)
	mi.s.faceNormal = append([]geom.Vec3(nil), h.FaceNormalsMut()...)
	mi.s.properties = append([]float64(nil), h.Properties()...)
	mi.s.halfedge = mesh.NewHalfedges(
		append([]int32(nil), h.HalfedgeStartsRO()...),
		append([]int32(nil), h.HalfedgePairsRO()...),
		append([]int32(nil), h.HalfedgePropsRO()...),
	)
	mi.s.halfedgeTangent = tangentsFromFlat(h.HalfedgeTangents())
	minB, maxB := h.GetBBox()
	mi.s.bBox = geom.Box{Min: minB, Max: maxB}
	mi.s.numProp = h.NumProp()
	mi.s.epsilon = h.GetEpsilon()
	mi.s.tolerance = h.GetTolerance()

	mi.s.meshRelation.TriRef = mi.s.meshRelation.TriRef[:0]
	for _, r := range h.TriRefs() {
		mi.s.meshRelation.TriRef = append(mi.s.meshRelation.TriRef,
			mesh.TriRef{MeshID: int(r.MeshID), OriginalID: int(r.OriginalID), FaceID: int(r.FaceID), CoplanarID: int(r.CoplanarID)})
	}
	mi.s.meshRelation.MeshIDTransform.Clear()
	for _, r := range h.MeshIDTransforms() { // ascending-key (std::map) order
		mi.s.meshRelation.MeshIDTransform.Set(int(r.MeshID), mesh.Relation{
			OriginalID: int(r.OriginalID), Transform: geom.Mat3x4(r.Transform),
			BackSide: r.BackSide, HasNormals: r.HasNormals,
		})
	}
	mi.s.meshRelation.OriginalID = keepOriginalID
	mi.s.status = keepStatus
}

// runCppAlgo is the differential-test harness (replacing the old MutableImpl
// runBridgeAlgo): marshal mi's native storage into a transient bridge handle, run a
// C++ pass against it, then reload the mutated storage back into mi. The collider is
// invalidated (Subdivide/Refine change geometry) so a later ensureCollider rebuilds.
func runCppAlgo(mi *MutableImpl, algo func(*cppref.MutableImpl)) {
	bm := cppref.NewMutableImpl()
	defer bm.Delete()
	marshalImplStorageToBridge(mi.s, bm)
	algo(bm)
	reloadFromBridge(mi, bm)
	mi.coll = nil
}
