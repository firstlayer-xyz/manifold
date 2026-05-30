package manifold

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
)

// implStorage owns everything C++ Manifold::Impl owns (src/impl.h:56-89). During
// the native-Impl-storage migration, manifold.Impl / MutableImpl become views over
// one implStorage; their getters/setters route through it. getImpl marshals a C++
// Impl into an implStorage and ToManifold marshals one back out (the transitional
// bridge seam, removed when the CSG tree is native). The collider is held on the
// view structs (Phase 1), not here, until Phase 5.
type implStorage struct {
	vertPos         []geom.Vec3
	halfedge        mesh.Halfedges
	properties      []float64
	vertNormal      []geom.Vec3
	faceNormal      []geom.Vec3
	halfedgeTangent []geom.Vec4
	meshRelation    mesh.MeshRelationD
	numProp         int
	bBox            geom.Box
	epsilon         float64
	tolerance       float64
	status          Error
}

// newImplStorage returns an empty storage with the C++ default scalars (bBox =
// empty, epsilon/tolerance = -1, originalID = -1).
func newImplStorage() *implStorage {
	return &implStorage{
		meshRelation: mesh.NewMeshRelationD(),
		bBox:         geom.EmptyBox(),
		epsilon:      -1,
		tolerance:    -1,
	}
}

// marshalImplStorageFromBridge reads a C++ Impl (via the bridge accessors) into a
// freshly-owned native implStorage. Slices are copied so the storage is
// independent of the bridge handle's lifetime.
func marshalImplStorageFromBridge(h *bridge.Impl) *implStorage {
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
func marshalImplStorageToBridge(s *implStorage, mi *bridge.MutableImpl) {
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

// reloadFromBridge re-reads the storage a still-C++ algorithm (Refine/Subdivide)
// just mutated on the bridge handle back into the native implStorage, so the
// native side stays the source of truth. originalID and status are preserved
// from s — Refine/Subdivide refine geometry but never change the mesh's identity
// or error state, and the bridge MutableImpl exposes no getters for them.
func (mi *MutableImpl) reloadFromBridge() {
	h := mi.h
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

// resizeVec3 grows/shrinks a []geom.Vec3 to length n (new slots zero-valued),
// reusing the backing array on shrink — the Vec semantics MutableImpl's
// Resize{Verts,VertNormals,FaceNormals} accessors need.
func resizeVec3(s []geom.Vec3, n int) []geom.Vec3 {
	if n <= len(s) {
		return s[:n]
	}
	return append(s, make([]geom.Vec3, n-len(s))...)
}

// triRefsToBridge converts the native triRef array to the bridge-shaped slice
// the (transitional) accessor callers expect.
func triRefsToBridge(refs []mesh.TriRef) []bridge.TriRef {
	out := make([]bridge.TriRef, len(refs))
	for i, r := range refs {
		out[i] = bridge.TriRef{MeshID: int32(r.MeshID), OriginalID: int32(r.OriginalID), FaceID: int32(r.FaceID), CoplanarID: int32(r.CoplanarID)}
	}
	return out
}

// meshIDTransformsToBridge returns the meshIDtransform entries in ascending
// meshID order — the native map is insertion-ordered, but C++ iterates the
// std::map by key, so callers must see key order.
func meshIDTransformsToBridge(mr *mesh.MeshRelationD) []bridge.MeshIDRelation {
	keys := append([]int(nil), mr.MeshIDTransform.Keys()...)
	sort.Ints(keys)
	out := make([]bridge.MeshIDRelation, 0, len(keys))
	for _, k := range keys {
		r, _ := mr.MeshIDTransform.Get(k)
		out = append(out, bridge.MeshIDRelation{
			MeshID: int32(k), OriginalID: int32(r.OriginalID),
			Transform: [4][3]float64(r.Transform), BackSide: r.BackSide, HasNormals: r.HasNormals,
		})
	}
	return out
}

// tangentsFromFlat unpacks the bridge's flat 4-per-element tangent buffer into
// Vec4s; tangentsToFlat is the inverse.
func tangentsFromFlat(flat []float64) []geom.Vec4 {
	out := make([]geom.Vec4, len(flat)/4)
	for i := range out {
		out[i] = geom.Vec4{X: flat[4*i], Y: flat[4*i+1], Z: flat[4*i+2], W: flat[4*i+3]}
	}
	return out
}

func tangentsToFlat(tangents []geom.Vec4) []float64 {
	flat := make([]float64, 4*len(tangents))
	for i, t := range tangents {
		flat[4*i], flat[4*i+1], flat[4*i+2], flat[4*i+3] = t.X, t.Y, t.Z, t.W
	}
	return flat
}
