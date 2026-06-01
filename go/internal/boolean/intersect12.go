package boolean

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// intersections is the Go port of the Intersections struct (boolean3.h:52): the
// edge<->face crossings — p1q2[k] = {P-index, Q-index}, the crossing count x12,
// and the 3D crossing point v12.
type intersections struct {
	p1q2 [][2]int
	x12  []int
	v12  []geom.Vec3
}

// emptyBox is the sentinel CollisionsBox skips (Min.X == +Inf), matching the
// C++ default-constructed Box() returned by Intersect12_'s f for reverse
// halfedges.
func emptyBox() geom.Box {
	inf := math.Inf(1)
	return geom.Box{
		Min: geom.Vec3{X: inf, Y: inf, Z: inf},
		Max: geom.Vec3{X: -inf, Y: -inf, Z: -inf},
	}
}

// intersect12 is the Go port of Intersect12_<expandP, forward> (boolean3.cpp:361):
// every edge(P)/face(Q) (forward) or edge(Q)/face(P) (!forward) crossing, found
// by querying the face mesh's collider with the edge mesh's edge boxes, then
// sorted by the edge index. bColl is the collider of the face mesh b
// (b = forward ? inQ : inP). SEQ path: a single accumulator (the C++ parallel
// tbb::combinable + concat is the later hardening step).
func intersect12(inP, inQ *mesh, bColl *collider.Collider, expandP, forward bool) intersections {
	k12 := newKernel12(inP, inQ, expandP, forward)
	a := k12.inA // edge mesh (= forward ? inP : inQ)

	// f: edge box of a for each halfedge (empty for reverse halfedges, so each
	// undirected edge is queried once).
	edgeBoxes := make([]geom.Box, len(a.halfedge.starts))
	for i := range edgeBoxes {
		s, e := a.halfedge.Start(i), a.halfedge.End(i)
		if s < e {
			edgeBoxes[i] = geom.NewBox(a.vertPos[s], a.vertPos[e])
		} else {
			edgeBoxes[i] = emptyBox()
		}
	}

	var result intersections
	bColl.CollisionsBox(edgeBoxes, false, false, func(queryIdx, leafIdx int) {
		x12, v12 := k12.call(queryIdx, leafIdx)
		if isFinite(v12.X) {
			pq := [2]int{queryIdx, leafIdx} // forward: {edgeP, faceQ}
			if !forward {
				pq = [2]int{leafIdx, queryIdx} // !forward: {faceP, edgeQ}
			}
			result.p1q2 = append(result.p1q2, pq)
			result.x12 = append(result.x12, x12)
			result.v12 = append(result.v12, v12)
		}
	})

	// Sort according to edges: index is the edge slot in p1q2 (0 forward, 1 not).
	index := 0
	if !forward {
		index = 1
	}
	// Mirror C++ default policies (boolean3.cpp:387-395): sequence/gather(Permute)
	// default to 1e5, stable_sort to 1e4. StableSort is stable and Permute is a
	// disjoint gather, so all stay bit-identical to the serial result.
	i12 := make([]int32, len(result.p1q2))
	parallel.Sequence(parallel.AutoPolicy(len(i12), 100000), i12)
	parallel.StableSort(parallel.AutoPolicy(len(i12), 10000), i12, func(x, y int32) bool {
		pa, pb := result.p1q2[x], result.p1q2[y]
		return pa[index] < pb[index] ||
			(pa[index] == pb[index] && pa[1-index] < pb[1-index])
	})
	result.p1q2 = parallel.Permute(parallel.AutoPolicy(len(i12), 100000), result.p1q2, i12)
	result.x12 = parallel.Permute(parallel.AutoPolicy(len(i12), 100000), result.x12, i12)
	result.v12 = parallel.Permute(parallel.AutoPolicy(len(i12), 100000), result.v12, i12)
	return result
}
