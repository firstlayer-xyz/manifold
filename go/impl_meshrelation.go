package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// markAllMeshIDHasNormals is the Go port of the per-meshID
// hasNormals=true loop used by Manifold::CalculateNormals
// (src/manifold.cpp). Iterates meshRelation_.meshIDtransform and
// sets every Relation.hasNormals to true.
//
// Implementation reads the map via the MutableImpl accessor, then
// clears + re-adds with the bit flipped.
func markAllMeshIDHasNormals(mi *bridge.MutableImpl) {
	rels := mi.MeshIDTransforms()
	mi.ClearMeshIDTransforms()
	for _, r := range rels {
		mi.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID),
			r.Transform, r.BackSide, true)
	}
}

// initializeOriginal is the Go port of C++
// Manifold::Impl::InitializeOriginal (src/impl.cpp).
//
// For each tri in meshRelation_.triRef, assign meshID, originalID, and
// faceID=-1, then re-anchor meshIDtransform with a single identity-
// transform entry tagged with the current AllHaveNormals state.
//
// numTri is the triangle count of the impl (we read it from the
// caller's already-known halfedge layout to avoid a round-trip).
//
// hadNormals comes from AllHaveNormals(). For the fresh-impl case
// (newImplFromShape), meshIDtransform is empty, so AllHaveNormals
// returns false. Callers that need the AsOriginal flow must read it
// from the impl first and pass the actual value.
//
// coplanarID semantics: the C++ code reads the existing coplanarID
// from each TriRef and preserves it. For a fresh impl (triRef was
// just resized from empty), that read is uninitialized memory; the
// downstream SetNormalsAndCoplanar overwrites it to -1 immediately
// in its first statement. We initialize coplanarID = -1 here for
// determinism — it's overwritten anyway.
func initializeOriginal(mi *bridge.MutableImpl, numTri int, hadNormals bool) {
	meshID := int(bridge.ImplReserveIDs(1))
	mi.SetMeshRelationOriginalID(meshID)

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
	mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)

	mi.ClearMeshIDTransforms()
	identity := [4][3]float64{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{0, 0, 0},
	}
	mi.AddMeshIDTransform(meshID, meshID, identity, false, hadNormals)
}
