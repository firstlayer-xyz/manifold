package manifold

import "github.com/firstlayer-xyz/manifold/go/bridge"

// InitializeOriginal is the Go port of C++
// Manifold::Impl::InitializeOriginal (src/impl.cpp).
//
// For each tri in meshRelation_.triRef, assign meshID, originalID,
// and faceID=-1, then re-anchor meshIDtransform with a single
// identity-transform entry tagged with the current AllHaveNormals
// state (read before clearing the map, matching the C++ order).
//
// coplanarID semantics: the C++ code reads the existing coplanarID
// from each TriRef and preserves it. For a fresh impl (triRef was
// just resized from empty), that read is uninitialized memory; the
// downstream SetNormalsAndCoplanar overwrites it to -1 immediately
// in its first statement. We initialize coplanarID = -1 here for
// determinism — it's overwritten anyway.
func (mi *MutableImpl) InitializeOriginal() {
	meshID := int(bridge.ImplReserveIDs(1))
	mi.h.SetMeshRelationOriginalID(meshID)

	numTri := mi.NumTri()
	meshIDs := make([]int32, numTri)
	originalIDs := make([]int32, numTri)
	faceIDs := make([]int32, numTri)
	coplanarIDs := make([]int32, numTri)
	for i := 0; i < numTri; i++ {
		meshIDs[i] = int32(meshID)
		originalIDs[i] = int32(meshID)
		faceIDs[i] = -1
		coplanarIDs[i] = -1
	}
	mi.h.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)

	// Mirror C++: AllHaveNormals is read BEFORE the meshIDtransform
	// map is cleared, so that the new single entry can carry the
	// inherited hasNormals state.
	hadNormals := mi.AllHaveNormals()
	mi.h.ClearMeshIDTransforms()
	identity := [4][3]float64{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{0, 0, 0},
	}
	mi.h.AddMeshIDTransform(meshID, meshID, identity, false, hadNormals)
}

// MarkAllMeshIDHasNormals is the Go port of the per-meshID
// hasNormals=true loop used by Manifold::CalculateNormals
// (src/manifold.cpp). Sets every Relation.hasNormals = true.
func (mi *MutableImpl) MarkAllMeshIDHasNormals() {
	rels := mi.MeshIDTransforms()
	mi.h.ClearMeshIDTransforms()
	for _, r := range rels {
		mi.h.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID),
			r.Transform, r.BackSide, true)
	}
}
