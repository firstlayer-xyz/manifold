package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// gatherFaces is the Go port of the two-arg form of C++
// Manifold::Impl::GatherFaces (src/sort.cpp): copy a subset of `old`
// into the empty MutableImpl `dst`, picking the faces indexed by
// faceNew2Old.
//
// Mirrors the C++ body step by step:
//  1. Resize and gather triRef from old.
//  2. Copy meshIDtransform map.
//  3. Copy properties_ + numProp_ if old has props.
//  4. Resize and gather faceNormal_ if old has cached normals.
//  5. Build faceOld2New via scatter.
//  6. Resize halfedge_ (and halfedgeTangent_ if old has tangents).
//  7. For each new face, run the ReindexFace remap.
func gatherFaces(dst *bridge.MutableImpl, old *bridge.Impl, faceNew2Old []int32) {
	numTri := len(faceNew2Old)

	// 1. Gather triRef.
	oldTriRefs := old.TriRefs()
	if len(oldTriRefs) > 0 {
		meshIDs := make([]int32, numTri)
		originalIDs := make([]int32, numTri)
		faceIDs := make([]int32, numTri)
		coplanarIDs := make([]int32, numTri)
		for i, oldF := range faceNew2Old {
			r := oldTriRefs[oldF]
			meshIDs[i] = r.MeshID
			originalIDs[i] = r.OriginalID
			faceIDs[i] = r.FaceID
			coplanarIDs[i] = r.CoplanarID
		}
		dst.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}

	// 2. Copy meshIDtransform map (preserves all entries from old).
	dst.ClearMeshIDTransforms()
	for _, rel := range old.MeshIDTransforms() {
		dst.AddMeshIDTransform(int(rel.MeshID), int(rel.OriginalID),
			rel.Transform, rel.BackSide, rel.HasNormals)
	}

	// 3. Copy properties_ + numProp_ if old has any.
	oldScalars := old.Scalars()
	if oldScalars.NumProp > 0 {
		dst.SetNumProp(oldScalars.NumProp)
		dst.SetProperties(append([]float64(nil), old.Properties()...))
	}

	// 4. Gather faceNormal_ if old has cached normals.
	oldFaceNormals := old.FaceNormals()
	oldNumTri := old.HalfedgeCount() / 3
	if len(oldFaceNormals) == oldNumTri && oldNumTri > 0 {
		dst.ResizeFaceNormals(numTri)
		dstFN := dst.FaceNormalsMut()
		for i, oldF := range faceNew2Old {
			dstFN[i] = oldFaceNormals[oldF]
		}
	}

	// 5. Build faceOld2New permutation.
	faceOld2New := make([]int32, oldNumTri)
	for newF, oldF := range faceNew2Old {
		faceOld2New[oldF] = int32(newF)
	}

	// 6 & 7. Rebuild halfedge_ + halfedgeTangent_ via the ReindexFace
	// remap: for each new halfedge, copy from old and rewrite the
	// pairedHalfedge to point at the new face index.
	oldStarts := old.HalfedgeStarts()
	oldPairs := old.HalfedgePairs()
	oldProps := old.HalfedgeProps()
	oldTangents := old.HalfedgeTangents() // 4*N doubles

	numHalfedge := 3 * numTri
	starts := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	pairs := make([]int32, numHalfedge)
	var tangents []float64
	if len(oldTangents) > 0 {
		tangents = make([]float64, 4*numHalfedge)
	}
	for newFace := 0; newFace < numTri; newFace++ {
		oldFace := int(faceNew2Old[newFace])
		for i := 0; i < 3; i++ {
			oldEdge := 3*oldFace + i
			newEdge := 3*newFace + i
			starts[newEdge] = oldStarts[oldEdge]
			props[newEdge] = oldProps[oldEdge]
			pair := int(oldPairs[oldEdge])
			pairedFace := pair / 3
			offset := pair - 3*pairedFace
			pairs[newEdge] = int32(3*int(faceOld2New[pairedFace]) + offset)
			if tangents != nil {
				copy(tangents[4*newEdge:4*newEdge+4],
					oldTangents[4*oldEdge:4*oldEdge+4])
			}
		}
	}
	dst.SetHalfedgesRaw(starts, props, pairs)
	if tangents != nil {
		dst.SetHalfedgeTangents(tangents)
	}
}
