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
		mi.ResizeVerts(newNumVert)
	}
	copy(mi.Verts(), newVerts)

	vertNormals := mi.VertNormals()
	if len(vertNormals) == numVert {
		newVN := parallel.Permute(policy, vertNormals, vertNew2Old[:newNumVert])
		if newNumVert != numVert {
			mi.ResizeVertNormals(newNumVert)
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
	mi.SetHalfedgesRaw(starts, props, pairs)
}

// faceBoxMortonOf is the body of Impl::GetFaceBoxMorton (src/sort.cpp),
// shared by the const Impl and mutable MutableImpl facade methods (C++
// has a single const method; the Go facade splits the type, so the body
// lives in one free function operating on the raw arrays, mirroring the
// checkHalfedge convention).
//
// For each triangle it fills the parallel faceBox / faceMorton arrays
// (both length NumTri): the triangle's 3-vert bounding box and the
// centroid's Morton code against bBox. Removed tris (any halfedge pair
// < 0) get an empty box + NoMortonCode, sorting them to the tail.
//
// C++ uses for_each_n(autoPolicy(NumTri(), 1e5), ...). We mirror.
func faceBoxMortonOf(verts []geom.Vec3, starts, pairs []int32, bBox geom.Box) ([]geom.Box, []uint32) {
	numTri := len(starts) / 3
	faceBox := make([]geom.Box, numTri)
	faceMorton := make([]uint32, numTri)
	infPos := math.Inf(1)
	infNeg := math.Inf(-1)
	policy := parallel.AutoPolicy(numTri, 100000)
	parallel.ForEachN(policy, numTri, func(face int) {
		if pairs[3*face] < 0 {
			faceBox[face] = geom.Box{
				Min: geom.Vec3{X: infPos, Y: infPos, Z: infPos},
				Max: geom.Vec3{X: infNeg, Y: infNeg, Z: infNeg},
			}
			faceMorton[face] = geom.NoMortonCode
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
			box = box.UnionPoint(p)
		}
		center = center.Scale(1.0 / 3.0)
		faceBox[face] = box
		faceMorton[face] = geom.MortonCode(center, bBox)
	})
	return faceBox, faceMorton
}

// GetFaceBoxMorton is the Go port of Impl::GetFaceBoxMorton
// (src/sort.cpp) — the const-view form. Returns the parallel faceBox /
// faceMorton arrays (length NumTri), the same two vectors C++ passes to
// the Collider constructor.
func (i *Impl) GetFaceBoxMorton() ([]geom.Box, []uint32) {
	minB, maxB := i.BBox()
	return faceBoxMortonOf(i.Verts(), i.HalfedgeStarts(), i.HalfedgePairs(), geom.Box{Min: minB, Max: maxB})
}

// GetFaceBoxMorton is the mutable-view form, used by SortGeometry.
func (mi *MutableImpl) GetFaceBoxMorton() ([]geom.Box, []uint32) {
	minB, maxB := mi.BBox()
	return faceBoxMortonOf(mi.Verts(), mi.HalfedgeStarts(), mi.HalfedgePairs(), geom.Box{Min: minB, Max: maxB})
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
		mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}

	// Permute faceNormal_.
	oldFaceNormals := append([]geom.Vec3(nil), mi.FaceNormals()...)
	if len(oldFaceNormals) == oldNumTri && oldNumTri > 0 {
		newFN := parallel.Permute(policy, oldFaceNormals, faceNew2Old)
		mi.ResizeFaceNormals(numTri)
		dstFN := mi.FaceNormals()
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
	mi.SetHalfedgesRaw(starts, props, pairs)
	if tangents != nil {
		mi.SetHalfedgeTangents(tangents)
	}
}

// SortFaces is the Go port of Impl::SortFaces (src/sort.cpp).
// Stable-sorts triangle indices by Morton code, trims kNoCode tris
// from the end, then in-place permutes triRef / faceNormal_ /
// halfedge_ / halfedgeTangent_.
//
// Takes and returns the (faceBox, faceMorton) parallel arrays; the
// trimmed/permuted versions feed the Collider downstream. (C++ mutates
// the two Vecs in place via Permute; Go gathers into fresh buffers.)
//
// C++ uses parallel sequence + stable_sort + lower_bound (serial) +
// Permute. We mirror.
func (mi *MutableImpl) SortFaces(faceBox []geom.Box, faceMorton []uint32) ([]geom.Box, []uint32) {
	n := len(faceMorton)
	policy := parallel.AutoPolicy(n, 100000)
	faceNew2Old := make([]int32, n)
	parallel.Sequence(policy, faceNew2Old)
	// stable_sort default threshold 1e4 (parallel.h:1140), not the
	// outer SortFaces policy's 1e5.
	parallel.StableSort(parallel.AutoPolicy(n, 10000), faceNew2Old, func(a, b int32) bool {
		return faceMorton[a] < faceMorton[b]
	})
	// Tris flagged for removal sorted to the end (NoMortonCode); trim.
	newNumTri := n
	for i, f := range faceNew2Old {
		if faceMorton[f] >= geom.NoMortonCode {
			newNumTri = i
			break
		}
	}
	faceNew2Old = faceNew2Old[:newNumTri]

	mi.GatherFacesInPlace(faceNew2Old)

	// Permute(faceMorton, faceNew2Old), Permute(faceBox, faceNew2Old) —
	// parallel gather into fresh buffers.
	box := make([]geom.Box, newNumTri)
	morton := make([]uint32, newNumTri)
	parallel.ForEachN(policy, newNumTri, func(newF int) {
		oldF := faceNew2Old[newF]
		box[newF] = faceBox[oldF]
		morton[newF] = faceMorton[oldF]
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
	faceBox, faceMorton := mi.GetFaceBoxMorton()
	box, morton := mi.SortFaces(faceBox, faceMorton)
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
	// BuildCollider refreshed the bridge bBox_ from the new collider's bounding
	// box (C++ SortGeometry: bBox_ = collider_.GetBoundingBox()); SortVerts may
	// have trimmed NaN verts, so pull the refreshed box back into native storage.
	minB, maxB := mi.h.GetBBox()
	mi.SetBBox(minB, maxB)
	mi.CompactProps()
}
