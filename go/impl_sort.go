package manifold

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

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
	mi.ReindexVerts(vertNew2Old, numVert)

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

// sortGeometry is the Go port of C++ Manifold::Impl::SortGeometry. It
// runs SortVerts in Go and delegates the GetFaceBoxMorton /
// SortFaces / Collider construction / bBox refresh / CompactProps
// remainder to the C++ side via SortGeometryPostVert.
//
// The early-out (empty mesh) is checked twice in C++; we mirror the
// first check here. The second check (after sort) is inside the C++
// post-vert helper.
func sortGeometry(mi *bridge.MutableImpl) {
	if len(mi.HalfedgeStartsRO()) == 0 {
		return
	}
	sortVerts(mi)
	mi.SortGeometryPostVert()
}
