package boolean

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/disjointsets"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// edgeIsBroken reports whether `edge` (of the vert mesh) has a crossing recorded
// in the sorted p1q2 (a crossing makes the edge "broken" — its endpoints are not
// united). Mirrors the C++ std::lower_bound on p1q2 by the [index] slot.
func edgeIsBroken(p1q2 [][2]int, edge, index int) bool {
	lo := sort.Search(len(p1q2), func(k int) bool { return p1q2[k][index] >= edge })
	return lo < len(p1q2) && p1q2[lo][index] == edge
}

// winding03 is the Go port of Winding03_<expandP, forward> (boolean3.cpp:409):
// the per-vertex winding number of mesh a's vertices relative to mesh b. Edges
// of a that are NOT broken (no recorded crossing) unite their endpoints in a
// DisjointSets; each resulting component's representative gets a Kernel02
// point-query against b's faces, and the winding is then flood-filled to the
// rest of the component. bColl is b's face collider; p1q2 is the matching
// Intersect12 output (sorted by the edge slot). SEQ path.
func winding03(inP, inQ *mesh, p1q2 [][2]int, bColl *collider.Collider, expandP, forward bool) []int {
	a, b := inP, inQ
	if !forward {
		a, b = inQ, inP
	}
	index := 0
	if !forward {
		index = 1
	}

	uA := disjointsets.New(len(a.vertPos))
	for edge := 0; edge < len(a.halfedge.starts); edge++ {
		start := a.halfedge.Start(edge)
		end := a.halfedge.End(edge)
		if start >= end {
			continue
		}
		if !edgeIsBroken(p1q2, edge, index) {
			uA.Unite(start, end)
		}
	}

	// Find components (roots). Order is irrelevant: w03 is written per-root then
	// flood-filled, mirroring the C++ unordered_set<root>.
	componentSet := make(map[int]struct{}, len(a.vertPos))
	for v := 0; v < len(a.vertPos); v++ {
		componentSet[uA.Find(v)] = struct{}{}
	}
	verts := make([]int, 0, len(componentSet))
	for c := range componentSet {
		verts = append(verts, c)
	}

	w03 := make([]int, len(a.vertPos))
	k02 := kernel02{inA: a, inB: b, expandP: expandP, forward: forward}
	queryPoints := make([]geom.Vec3, len(verts))
	for i, v := range verts {
		queryPoints[i] = a.vertPos[v]
	}
	bColl.CollisionsPoint(queryPoints, false, false, func(i, faceB int) {
		s02, z02 := k02.call(verts[i], faceB)
		// i is distinct per query and verts are unique, so no atomics needed.
		if isFinite(z02) {
			sign := 1
			if !forward {
				sign = -1
			}
			w03[verts[i]] += s02 * sign
		}
	})

	// Flood fill the component winding to every member.
	for i := 0; i < len(w03); i++ {
		root := uA.Find(i)
		if root == i {
			continue
		}
		w03[i] = w03[root]
	}
	return w03
}
