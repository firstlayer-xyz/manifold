package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// SortVerts is the Go port of C++ Manifold::Impl::SortVerts
// (src/sort.cpp): reorder vertPos_ + vertNormal_ by per-vert Morton
// code, then update halfedge_ to point at the new vert indices.
//
// Verts whose Morton code is NoMortonCode (NaN positions) sort to the
// end and are trimmed; this matches the C++ behavior of dropping
// unreferenced verts that were marked NaN upstream.
//
// C++ uses policy = autoPolicy(numVert, 1e5) and runs the Morton fill
// + stable_sort + the final Permutes through tbb's parallel
// primitives. We mirror via parallel.{ForEachN,StableSort,Permute}.
func (mi *MutableImpl) SortVerts() {
	verts := mi.Verts()
	numVert := len(verts)
	if numVert == 0 {
		return
	}
	minV, maxV := mi.BBox()
	bBox := geom.Box{Min: minV, Max: maxV}
	policy := parallel.AutoPolicy(numVert, 100000)

	vertMorton := make([]uint32, numVert)
	parallel.ForEachN(policy, numVert, func(v int) {
		vertMorton[v] = geom.MortonCode(verts[v], bBox)
	})

	vertNew2Old := make([]int32, numVert)
	parallel.Sequence(policy, vertNew2Old)
	// C++ stable_sort with no policy form defaults to threshold 1e4
	// (parallel.h:1140); SortVerts's outer policy uses 1e5 for the
	// for_each_n calls but the stable_sort uses its own default.
	parallel.StableSort(parallel.AutoPolicy(numVert, 10000), vertNew2Old, func(a, b int32) bool {
		return vertMorton[a] < vertMorton[b]
	})

	mi.ReindexVerts(vertNew2Old, numVert)

	// lower_bound: the C++ uses std::lower_bound serially since the
	// trim point comes from the sorted array's transition. Tail of the
	// sorted vertNew2Old has the NoMortonCode entries — locate the
	// first one and that is newNumVert.
	newNumVert := numVert
	for i, vIdx := range vertNew2Old {
		if vertMorton[vIdx] >= geom.NoMortonCode {
			newNumVert = i
			break
		}
	}

	// Permute(vertPos_, vertNew2Old): C++ shrinks vertPos_ to
	// newNumVert size by gather-into-fresh + resize. We mirror via
	// parallel.Permute and copy back.
	newVerts := parallel.Permute(policy, verts, vertNew2Old[:newNumVert])
	if newNumVert != numVert {
		mi.h.ResizeVerts(newNumVert)
	}
	copy(mi.Verts(), newVerts)

	vertNormals := mi.VertNormals()
	if len(vertNormals) == numVert {
		newVN := parallel.Permute(policy, vertNormals, vertNew2Old[:newNumVert])
		if newNumVert != numVert {
			mi.h.ResizeVertNormals(newNumVert)
		}
		copy(mi.VertNormals(), newVN)
	}
}

// ReindexVerts is the Go port of C++ Manifold::Impl::ReindexVerts
// (src/sort.cpp): for every halfedge with startVert >= 0, rewrite
// its start to vertOld2New[startVert] (and, when the impl has no
// extra properties, the propVert too).
//
// C++ uses parallel.Scatter for the vertOld2New build and
// for_each_n(autoPolicy(oldNumVert, 1e5), countAt(0), halfedge_.size())
// for the halfedge rewrite. We mirror exactly.
func (mi *MutableImpl) ReindexVerts(vertNew2Old []int32, oldNumVert int) {
	policy := parallel.AutoPolicy(oldNumVert, 100000)
	vertOld2New := make([]int32, oldNumVert)
	// scatter(countAt(0), countAt(NumVert()), vertNew2Old, vertOld2New)
	// is equivalent to vertOld2New[vertNew2Old[i]] = i for every i.
	parallel.ForEachN(policy, len(vertNew2Old), func(i int) {
		vertOld2New[vertNew2Old[i]] = int32(i)
	})
	hasProp := mi.NumProp() > 0

	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	props := append([]int32(nil), mi.HalfedgeProps()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	parallel.ForEachN(policy, len(starts), func(i int) {
		s := starts[i]
		if s < 0 {
			return
		}
		newStart := vertOld2New[s]
		starts[i] = newStart
		if !hasProp {
			props[i] = newStart
		}
	})
	mi.h.SetHalfedgesRaw(starts, props, pairs)
}

// faceBoxMorton holds the per-face bounding box + Morton code that
// Impl::GetFaceBoxMorton produces.
type faceBoxMorton struct {
	box    geom.Box
	morton uint32
}

// GetFaceBoxMorton is the Go port of Impl::GetFaceBoxMorton
// (src/sort.cpp). For each triangle: compute its 3-vert bounding
// box, take the centroid's Morton code against the mesh bBox.
// Removed-tri sentinel (any halfedge pair < 0) maps to NoMortonCode.
//
// C++ uses for_each_n(autoPolicy(NumTri(), 1e5), ...). We mirror.
func (mi *MutableImpl) GetFaceBoxMorton() []faceBoxMorton {
	verts := mi.Verts()
	starts := mi.HalfedgeStarts()
	pairs := mi.HalfedgePairs()
	minB, maxB := mi.BBox()
	bBox := geom.Box{Min: minB, Max: maxB}
	numTri := len(starts) / 3
	out := make([]faceBoxMorton, numTri)
	infPos := math.Inf(1)
	infNeg := math.Inf(-1)
	policy := parallel.AutoPolicy(numTri, 100000)
	parallel.ForEachN(policy, numTri, func(face int) {
		if pairs[3*face] < 0 {
			out[face].morton = geom.NoMortonCode
			return
		}
		box := geom.Box{
			Min: geom.Vec3{X: infPos, Y: infPos, Z: infPos},
			Max: geom.Vec3{X: infNeg, Y: infNeg, Z: infNeg},
		}
		var center geom.Vec3
		for i := 0; i < 3; i++ {
			p := verts[starts[3*face+i]]
			center = center.Add(p)
			if p.X < box.Min.X {
				box.Min.X = p.X
			}
			if p.Y < box.Min.Y {
				box.Min.Y = p.Y
			}
			if p.Z < box.Min.Z {
				box.Min.Z = p.Z
			}
			if p.X > box.Max.X {
				box.Max.X = p.X
			}
			if p.Y > box.Max.Y {
				box.Max.Y = p.Y
			}
			if p.Z > box.Max.Z {
				box.Max.Z = p.Z
			}
		}
		center = center.Scale(1.0 / 3.0)
		out[face].box = box
		out[face].morton = geom.MortonCode(center, bBox)
	})
	return out
}

// GatherFacesInPlace permutes meshRelation_.triRef, faceNormal_, and
// halfedge_/halfedgeTangent_ in place by faceNew2Old. Mirrors the
// single-arg C++ Impl::GatherFaces (src/sort.cpp) — used by SortFaces.
//
// C++ uses policy = autoPolicy(numTri, 1e5), parallel.Scatter for
// faceOld2New, parallel.Permute for triRef and faceNormal, and
// for_each_n for the per-face halfedge reindex (ReindexFace functor).
func (mi *MutableImpl) GatherFacesInPlace(faceNew2Old []int32) {
	numTri := len(faceNew2Old)
	oldNumTri := len(mi.HalfedgeStarts()) / 3
	policy := parallel.AutoPolicy(numTri, 100000)

	// Permute triRef.
	oldTriRefs := mi.TriRefs()
	if len(oldTriRefs) == oldNumTri {
		permuted := parallel.Permute(policy, oldTriRefs, faceNew2Old)
		meshIDs := make([]int32, numTri)
		originalIDs := make([]int32, numTri)
		faceIDs := make([]int32, numTri)
		coplanarIDs := make([]int32, numTri)
		parallel.ForEachN(policy, numTri, func(i int) {
			r := permuted[i]
			meshIDs[i] = r.MeshID
			originalIDs[i] = r.OriginalID
			faceIDs[i] = r.FaceID
			coplanarIDs[i] = r.CoplanarID
		})
		mi.h.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}

	// Permute faceNormal_.
	oldFaceNormals := append([]geom.Vec3(nil), mi.FaceNormals()...)
	if len(oldFaceNormals) == oldNumTri && oldNumTri > 0 {
		newFN := parallel.Permute(policy, oldFaceNormals, faceNew2Old)
		mi.h.ResizeFaceNormals(numTri)
		dstFN := mi.h.FaceNormalsMut()
		copy(dstFN, newFN)
	}

	// scatter(countAt(0), countAt(numTri), faceNew2Old, faceOld2New)
	// — write faceOld2New[faceNew2Old[i]] = i for every i.
	faceOld2New := make([]int32, oldNumTri)
	parallel.ForEachN(policy, numTri, func(i int) {
		faceOld2New[faceNew2Old[i]] = int32(i)
	})

	oldStarts := append([]int32(nil), mi.HalfedgeStarts()...)
	oldPairs := append([]int32(nil), mi.HalfedgePairs()...)
	oldProps := append([]int32(nil), mi.HalfedgeProps()...)
	oldTangents := append([]float64(nil), mi.HalfedgeTangents()...)

	numHalfedge := 3 * numTri
	starts := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	pairs := make([]int32, numHalfedge)
	var tangents []float64
	if len(oldTangents) > 0 {
		tangents = make([]float64, 4*numHalfedge)
	}
	// for_each_n over numTri — the ReindexFace functor body, applied
	// per-new-face. Each iteration writes to a disjoint range, no race.
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
	mi.h.SetHalfedgesRaw(starts, props, pairs)
	if tangents != nil {
		mi.h.SetHalfedgeTangents(tangents)
	}
}

// SortFaces is the Go port of Impl::SortFaces (src/sort.cpp).
// Stable-sorts triangle indices by Morton code, trims kNoCode tris
// from the end, then in-place permutes triRef / faceNormal_ /
// halfedge_ / halfedgeTangent_.
//
// Returns the permuted (faceBox, faceMorton) parallel arrays so the
// Collider can consume them downstream.
//
// C++ uses parallel sequence + stable_sort + lower_bound (serial) +
// Permute. We mirror.
func (mi *MutableImpl) SortFaces(faces []faceBoxMorton) ([]geom.Box, []uint32) {
	n := len(faces)
	policy := parallel.AutoPolicy(n, 100000)
	idx := make([]int32, n)
	parallel.Sequence(policy, idx)
	// stable_sort default threshold 1e4 (parallel.h:1140), not the
	// outer SortFaces policy's 1e5.
	parallel.StableSort(parallel.AutoPolicy(n, 10000), idx, func(a, b int32) bool {
		return faces[a].morton < faces[b].morton
	})
	newNumTri := n
	for i, f := range idx {
		if faces[f].morton >= geom.NoMortonCode {
			newNumTri = i
			break
		}
	}
	idx = idx[:newNumTri]

	mi.GatherFacesInPlace(idx)

	// Permute(faceMorton, idx), Permute(faceBox, idx) — parallel
	// gather into fresh buffers.
	box := make([]geom.Box, newNumTri)
	morton := make([]uint32, newNumTri)
	parallel.ForEachN(policy, newNumTri, func(newF int) {
		oldF := idx[newF]
		box[newF] = faces[oldF].box
		morton[newF] = faces[oldF].morton
	})
	return box, morton
}

// SortGeometry is the Go port of C++ Manifold::Impl::SortGeometry.
// SortVerts, GetFaceBoxMorton, SortFaces (with in-place GatherFaces),
// and CompactProps all run in Go. Only the Collider AABB-tree build
// + bBox refresh remain in C++ via the BuildCollider bridge.
func (mi *MutableImpl) SortGeometry() {
	if len(mi.HalfedgeStarts()) == 0 {
		mi.h.BuildCollider(nil, nil)
		return
	}
	mi.SortVerts()
	faces := mi.GetFaceBoxMorton()
	box, morton := mi.SortFaces(faces)
	if len(mi.HalfedgeStarts()) == 0 {
		mi.h.BuildCollider(nil, nil)
		return
	}
	flat := make([]float64, 6*len(box))
	policy := parallel.AutoPolicy(len(box), 100000)
	parallel.ForEachN(policy, len(box), func(i int) {
		b := box[i]
		flat[6*i+0] = b.Min.X
		flat[6*i+1] = b.Min.Y
		flat[6*i+2] = b.Min.Z
		flat[6*i+3] = b.Max.X
		flat[6*i+4] = b.Max.Y
		flat[6*i+5] = b.Max.Z
	})
	mi.h.BuildCollider(flat, morton)
	mi.CompactProps()
}
