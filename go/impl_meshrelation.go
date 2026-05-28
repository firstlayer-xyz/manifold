package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// InitializeOriginal is the Go port of C++
// Manifold::Impl::InitializeOriginal (src/impl.cpp).
//
// For each tri in meshRelation_.triRef, assign meshID, originalID,
// and faceID=-1, preserving the existing coplanarID. Then re-anchor
// meshIDtransform with a single identity-transform entry tagged with
// the current AllHaveNormals state (read before clearing the map,
// matching the C++ order).
//
// C++ uses for_each_n(autoPolicy(NumTri(), 1e5), countAt(0),
// NumTri(), ...). We mirror via parallel.ForEachN.
func (mi *MutableImpl) InitializeOriginal() {
	meshID := int(bridge.ImplReserveIDs(1))
	mi.h.SetMeshRelationOriginalID(meshID)

	numTri := mi.NumTri()
	oldTriRefs := mi.TriRefs() // may be empty if triRef was just resized
	meshIDs := make([]int32, numTri)
	originalIDs := make([]int32, numTri)
	faceIDs := make([]int32, numTri)
	coplanarIDs := make([]int32, numTri)
	policy := parallel.AutoPolicy(numTri, 100000)
	parallel.ForEachN(policy, numTri, func(tri int) {
		meshIDs[tri] = int32(meshID)
		originalIDs[tri] = int32(meshID)
		faceIDs[tri] = -1
		// Preserve existing coplanarID exactly like C++:
		// triRef[tri] = {meshID, meshID, -1, triRef[tri].coplanarID};
		if tri < len(oldTriRefs) {
			coplanarIDs[tri] = oldTriRefs[tri].CoplanarID
		} else {
			coplanarIDs[tri] = 0
		}
	})
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
//
// The C++ form is a serial `for (auto& m : meshRelation_.meshIDtransform)`
// over a std::map — small N (typically << 100), no parallelism.
// We mirror with a serial loop.
func (mi *MutableImpl) MarkAllMeshIDHasNormals() {
	rels := mi.MeshIDTransforms()
	mi.h.ClearMeshIDTransforms()
	for _, r := range rels {
		mi.h.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID),
			r.Transform, r.BackSide, true)
	}
}
