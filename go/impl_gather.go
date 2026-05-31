package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// GatherFaces is the Go port of the two-arg form of C++
// Manifold::Impl::GatherFaces (src/sort.cpp): copy a subset of old
// into this (empty) MutableImpl, picking the faces indexed by
// faceNew2Old.
//
// Mirrors the C++ body step by step:
//  1. Resize and gather triRef from old (parallel.Gather equivalent).
//  2. Copy meshIDtransform map (serial — small std::map).
//  3. Copy properties_ + numProp_ if old has props.
//  4. Resize and gather faceNormal_ if old has cached normals
//     (parallel.Gather).
//  5. Build faceOld2New via scatter (parallel.ForEachN equivalent).
//  6. Resize halfedge_ (and halfedgeTangent_ if old has tangents).
//  7. For each new face, run the ReindexFace remap (parallel
//     for_each_n).
//
// C++ uses policy = autoPolicy(numTri, 1e5).
func (mi *MutableImpl) GatherFaces(old *Impl, faceNew2Old []int32) {
	numTri := len(faceNew2Old)
	policy := parallel.AutoPolicy(numTri, 100000)

	// triRef gather: gather + repack into the four parallel arrays the
	// bridge expects.
	oldTriRefs := old.TriRefs()
	if len(oldTriRefs) > 0 {
		permuted := parallel.Permute(policy, oldTriRefs, faceNew2Old)
		meshIDs := make([]int32, numTri)
		originalIDs := make([]int32, numTri)
		faceIDs := make([]int32, numTri)
		coplanarIDs := make([]int32, numTri)
		parallel.ForEachN(policy, numTri, func(i int) {
			r := permuted[i]
			meshIDs[i] = int32(r.MeshID)
			originalIDs[i] = int32(r.OriginalID)
			faceIDs[i] = int32(r.FaceID)
			coplanarIDs[i] = int32(r.CoplanarID)
		})
		mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}

	// meshIDtransform copy — serial in C++, std::map iteration.
	mi.ClearMeshIDTransforms()
	for _, rel := range old.MeshIDTransforms() {
		mi.AddMeshIDTransform(rel.MeshID, rel.OriginalID,
			rel.Transform, rel.BackSide, rel.HasNormals)
	}

	oldScalars := old.Scalars()
	if oldScalars.NumProp > 0 {
		mi.SetNumProp(oldScalars.NumProp)
		mi.SetProperties(append([]float64(nil), old.Properties()...))
	}

	// faceNormal_ gather.
	oldFaceNormals := old.FaceNormals()
	oldNumTri := old.HalfedgeCount() / 3
	if len(oldFaceNormals) == oldNumTri && oldNumTri > 0 {
		newFN := parallel.Permute(policy, []geom.Vec3(oldFaceNormals), faceNew2Old)
		mi.ResizeFaceNormals(numTri)
		dstFN := mi.FaceNormals()
		copy(dstFN, newFN)
	}

	// scatter(countAt(0), countAt(numTri), faceNew2Old, faceOld2New)
	// — faceOld2New[faceNew2Old[i]] = i.
	faceOld2New := make([]int32, oldNumTri)
	parallel.ForEachN(policy, numTri, func(i int) {
		faceOld2New[faceNew2Old[i]] = int32(i)
	})

	oldStarts := old.HalfedgeStarts()
	oldPairs := old.HalfedgePairs()
	oldProps := old.HalfedgeProps()
	oldTangents := old.HalfedgeTangents()

	numHalfedge := 3 * numTri
	starts := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	pairs := make([]int32, numHalfedge)
	var tangents []float64
	if len(oldTangents) > 0 {
		tangents = make([]float64, 4*numHalfedge)
	}
	parallel.ForEachN(policy, numTri, func(newFace int) {
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
	})
	mi.SetHalfedgesRaw(starts, props, pairs)
	if tangents != nil {
		mi.SetHalfedgeTangents(tangents)
	}
}
