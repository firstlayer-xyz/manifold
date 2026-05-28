package manifold

import (
	"math"
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// faceBoxMorton holds the per-face bounding box + Morton code that
// Impl::GetFaceBoxMorton produces. Faces flagged for removal (whose
// halfedge[0].Pair < 0) get Morton == NoMortonCode so they sort to
// the end.
type faceBoxMorton struct {
	box    geom.Box
	morton uint32
}

// getFaceBoxMorton is the Go port of Impl::GetFaceBoxMorton
// (src/sort.cpp). For each triangle: compute its 3-vert bounding
// box, take the centroid's Morton code against the mesh bBox.
// Removed-tri sentinel (any halfedge pair < 0) maps to NoMortonCode.
func getFaceBoxMorton(mi *bridge.MutableImpl) []faceBoxMorton {
	verts := mi.Verts()
	starts := mi.HalfedgeStartsRO()
	pairs := mi.HalfedgePairsRO()
	minB, maxB := mi.GetBBox()
	bBox := geom.Box{Min: minB, Max: maxB}
	numTri := len(starts) / 3
	out := make([]faceBoxMorton, numTri)
	infPos := math.Inf(1)
	infNeg := math.Inf(-1)
	for face := 0; face < numTri; face++ {
		if pairs[3*face] < 0 {
			out[face].morton = geom.NoMortonCode
			continue
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
	}
	return out
}

// gatherFacesInPlace permutes meshRelation_.triRef, faceNormal_, and
// halfedge_/halfedgeTangent_ in place by faceNew2Old. Mirrors the
// single-arg C++ Impl::GatherFaces — used by SortFaces.
func gatherFacesInPlace(mi *bridge.MutableImpl, faceNew2Old []int32) {
	numTri := len(faceNew2Old)
	oldNumTri := len(mi.HalfedgeStartsRO()) / 3

	// 1. Permute triRef if it tracks numTri.
	oldTriRefs := mi.TriRefs()
	if len(oldTriRefs) == oldNumTri {
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
		mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}

	// 2. Permute faceNormal_ if it tracks numTri.
	oldFaceNormals := append([]geom.Vec3(nil), mi.FaceNormalsMut()...)
	if len(oldFaceNormals) == oldNumTri && oldNumTri > 0 {
		mi.ResizeFaceNormals(numTri)
		dstFN := mi.FaceNormalsMut()
		for i, oldF := range faceNew2Old {
			dstFN[i] = oldFaceNormals[oldF]
		}
	}

	// 3. Build faceOld2New permutation (size = old NumTri).
	faceOld2New := make([]int32, oldNumTri)
	for newF, oldF := range faceNew2Old {
		faceOld2New[oldF] = int32(newF)
	}

	// 4. Rebuild halfedge_ + halfedgeTangent_ via ReindexFace.
	oldStarts := append([]int32(nil), mi.HalfedgeStartsRO()...)
	oldPairs := append([]int32(nil), mi.HalfedgePairsRO()...)
	oldProps := append([]int32(nil), mi.HalfedgePropsRO()...)
	oldTangents := append([]float64(nil), mi.HalfedgeTangents()...)

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
	mi.SetHalfedgesRaw(starts, props, pairs)
	if tangents != nil {
		mi.SetHalfedgeTangents(tangents)
	}
}

// sortFaces is the Go port of Impl::SortFaces (src/sort.cpp).
// Stable-sorts triangle indices by Morton code, trims kNoCode tris
// from the end, then in-place permutes triRef / faceNormal_ /
// halfedge_ / halfedgeTangent_.
//
// Returns the permuted (faceBox, faceMorton) parallel arrays so the
// Collider can consume them downstream.
func sortFaces(mi *bridge.MutableImpl, faces []faceBoxMorton) ([]geom.Box, []uint32) {
	n := len(faces)
	idx := make([]int32, n)
	for i := range idx {
		idx[i] = int32(i)
	}
	sort.SliceStable(idx, func(a, b int) bool {
		return faces[idx[a]].morton < faces[idx[b]].morton
	})
	// Trim tris with NoMortonCode (removed) to the end and drop them.
	newNumTri := n
	for i, f := range idx {
		if faces[f].morton >= geom.NoMortonCode {
			newNumTri = i
			break
		}
	}
	idx = idx[:newNumTri]

	gatherFacesInPlace(mi, idx)

	box := make([]geom.Box, newNumTri)
	morton := make([]uint32, newNumTri)
	for newF, oldF := range idx {
		box[newF] = faces[oldF].box
		morton[newF] = faces[oldF].morton
	}
	return box, morton
}

// reindexVerts is the Go port of C++ Manifold::Impl::ReindexVerts
// (src/sort.cpp): for every halfedge with startVert >= 0, rewrite
// its start to vertOld2New[startVert] (and, when the impl has no
// extra properties, the propVert too).
//
// vertNew2Old has length newNumVert (the post-sort vertex count) and
// maps each new vert index back to its old position. oldNumVert is
// the size of the source vertex array before the sort. Together they
// let us build the inverse permutation vertOld2New that the halfedge
// rewrite needs.
func reindexVerts(mi *bridge.MutableImpl, vertNew2Old []int32, oldNumVert int) {
	vertOld2New := make([]int32, oldNumVert)
	for newV, oldV := range vertNew2Old {
		vertOld2New[oldV] = int32(newV)
	}
	hasProp := mi.NumProp() > 0

	starts := append([]int32(nil), mi.HalfedgeStartsRO()...)
	props := append([]int32(nil), mi.HalfedgePropsRO()...)
	pairs := append([]int32(nil), mi.HalfedgePairsRO()...)
	for i, s := range starts {
		if s < 0 {
			continue
		}
		newStart := vertOld2New[s]
		starts[i] = newStart
		if !hasProp {
			props[i] = newStart
		}
	}
	mi.SetHalfedgesRaw(starts, props, pairs)
}

// sortVerts is the Go port of C++ Manifold::Impl::SortVerts
// (src/sort.cpp): reorder vertPos_ + vertNormal_ by per-vert Morton
// code, then update halfedge_ to point at the new vert indices.
//
// Verts whose Morton code is NoMortonCode (NaN positions) sort to the
// end and are trimmed; this matches the C++ behavior of dropping
// unreferenced verts that were marked NaN upstream.
func sortVerts(mi *bridge.MutableImpl) {
	verts := mi.Verts()
	numVert := len(verts)
	if numVert == 0 {
		return
	}
	minV, maxV := mi.GetBBox()
	bBox := geom.Box{Min: minV, Max: maxV}

	// Per-vert Morton code.
	vertMorton := make([]uint32, numVert)
	for i, v := range verts {
		vertMorton[i] = geom.MortonCode(v, bBox)
	}

	// vertNew2Old: stable_sort indices by Morton code ascending.
	// NoMortonCode is 0xFFFFFFFF, so NaN-flagged verts land at the end.
	vertNew2Old := make([]int32, numVert)
	for i := range vertNew2Old {
		vertNew2Old[i] = int32(i)
	}
	sort.SliceStable(vertNew2Old, func(a, b int) bool {
		return vertMorton[vertNew2Old[a]] < vertMorton[vertNew2Old[b]]
	})

	// Update halfedge_ to point at the new vert indices.
	reindexVerts(mi, vertNew2Old, numVert)

	// lower_bound: first index whose vert has Morton == NoMortonCode.
	// All later verts get dropped.
	newNumVert := numVert
	for i, vIdx := range vertNew2Old {
		if vertMorton[vIdx] >= geom.NoMortonCode {
			newNumVert = i
			break
		}
	}

	// Permute vertPos_ in place. We can't shrink + write to the aliased
	// slice safely, so we permute via a Go-side buffer and copy back.
	newVerts := make([]geom.Vec3, newNumVert)
	for i := 0; i < newNumVert; i++ {
		newVerts[i] = verts[vertNew2Old[i]]
	}
	if newNumVert != numVert {
		mi.ResizeVerts(newNumVert)
	}
	copy(mi.Verts(), newVerts)

	// Permute vertNormal_ if it tracks vertPos_ (same length).
	vertNormals := mi.VertNormals()
	if len(vertNormals) == numVert {
		newVN := make([]geom.Vec3, newNumVert)
		for i := 0; i < newNumVert; i++ {
			newVN[i] = vertNormals[vertNew2Old[i]]
		}
		if newNumVert != numVert {
			mi.ResizeVertNormals(newNumVert)
		}
		copy(mi.VertNormals(), newVN)
	}
}

// sortGeometry is the Go port of C++ Manifold::Impl::SortGeometry. The
// algorithm is now almost entirely in Go: SortVerts, GetFaceBoxMorton,
// SortFaces (with in-place GatherFaces), and the post-sort empty
// check all run here. Only the Collider AABB-tree build + bBox refresh
// + CompactProps remain in C++, wrapped by BuildCollider.
//
// Mirrors the C++ body's two empty-mesh checks: one before any work,
// one after sorting (in case sort dropped all faces).
func sortGeometry(mi *bridge.MutableImpl) {
	if len(mi.HalfedgeStartsRO()) == 0 {
		mi.BuildCollider(nil, nil)
		return
	}
	sortVerts(mi)
	faces := getFaceBoxMorton(mi)
	box, morton := sortFaces(mi, faces)
	if len(mi.HalfedgeStartsRO()) == 0 {
		mi.BuildCollider(nil, nil)
		return
	}
	// Flatten box for the bridge ABI (6 doubles per face).
	flat := make([]float64, 6*len(box))
	for i, b := range box {
		flat[6*i+0] = b.Min.X
		flat[6*i+1] = b.Min.Y
		flat[6*i+2] = b.Min.Z
		flat[6*i+3] = b.Max.X
		flat[6*i+4] = b.Max.Y
		flat[6*i+5] = b.Max.Z
	}
	mi.BuildCollider(flat, morton)
}
