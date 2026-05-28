package manifold

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// createHalfedges is the Go port of the single-arg form of C++
// Manifold::Impl::CreateHalfedges (src/impl.cpp). triVerts is flat:
// 3 int32 indices per triangle. Builds halfedge_.start_, .propVert_,
// .paired_ and writes them into mi via bridge.SetHalfedgesRaw.
//
// Mirrors the C++ small-vertex-count branch (sort-based pairing). The
// duplicate-triangle detection path is mirrored as a panic for now —
// the inputs we currently feed in (primitive shape tables) are
// duplicate-free, and any future caller that violates that needs an
// explicit port of the C++ "body" lambda. Keeping the panic makes
// the gap explicit rather than silently wrong.
func createHalfedges(mi *bridge.MutableImpl, triVerts []int32) {
	numHalfedge := len(triVerts)
	if numHalfedge%3 != 0 {
		panic("createHalfedges: triVerts length must be a multiple of 3")
	}
	numTri := numHalfedge / 3
	numEdge := numHalfedge / 2

	// PrepHalfedges: build per-halfedge records and 64-bit sort keys.
	// Key layout (matching C++):
	//   bit 63 = 1 iff v0 < v1 (forward edge)
	//   bits 32..62 = min(v0, v1)
	//   bits 0..31 = max(v0, v1)
	type heRec struct {
		startVert, endVert, propVert int32
	}
	he := make([]heRec, numHalfedge)
	key := make([]uint64, numHalfedge)
	for tri := 0; tri < numTri; tri++ {
		for i := 0; i < 3; i++ {
			j := i + 1
			if j == 3 {
				j = 0
			}
			e := 3*tri + i
			v0 := triVerts[3*tri+i]
			v1 := triVerts[3*tri+j]
			if v0 == v1 {
				panic("createHalfedges: topological degeneracy (v0 == v1)")
			}
			// triVert is empty in the single-arg form, so propVert = v0.
			he[e] = heRec{startVert: v0, endVert: v1, propVert: v0}
			var fwd uint64
			minV, maxV := v0, v1
			if v0 < v1 {
				fwd = 1
			} else {
				minV, maxV = v1, v0
			}
			key[e] = (fwd << 63) | (uint64(uint32(minV)) << 32) | uint64(uint32(maxV))
		}
	}

	// Stable sort halfedge indices by their key. Stable sort matches
	// the C++ stable_sort used when vertCount < 1<<18.
	ids := make([]int, numHalfedge)
	for i := range ids {
		ids[i] = i
	}
	sort.SliceStable(ids, func(a, b int) bool {
		return key[ids[a]] < key[ids[b]]
	})

	// After sort: ids[0..numEdge) are backward halfedges (v0 >= v1),
	// ids[numEdge..2*numEdge) are forward halfedges. Each backward
	// edge at ids[i] pairs with the forward edge at ids[i+numEdge]
	// — both share the same (min, max) endpoints. This holds when
	// no triangles are duplicated (no two halfedges have identical
	// (startVert, endVert) AND identical opposite-vert).
	//
	// Detect the duplicate case explicitly and bail; the C++
	// "body" lambda handles it via reordering and the removed[]
	// mask, which we have not ported.
	for i := 1; i < numEdge; i++ {
		h0 := he[ids[i-1]]
		h1 := he[ids[i]]
		if h0.startVert == h1.startVert && h0.endVert == h1.endVert {
			panic("createHalfedges: duplicate halfedges detected — Go port currently handles only duplicate-free inputs")
		}
	}

	// Write halfedge_ arrays.
	starts := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	paireds := make([]int32, numHalfedge)
	for i := 0; i < numEdge; i++ {
		pair0 := ids[i]
		pair1 := ids[i+numEdge]
		starts[pair0] = he[pair0].startVert
		props[pair0] = he[pair0].propVert
		paireds[pair0] = int32(pair1)
		starts[pair1] = he[pair1].startVert
		props[pair1] = he[pair1].propVert
		paireds[pair1] = int32(pair0)
	}
	mi.SetHalfedgesRaw(starts, props, paireds)
}
