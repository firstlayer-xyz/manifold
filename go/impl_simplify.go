package manifold

import (
	"math"
	"runtime"
	"sync"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// flagStore is the Go port of the anonymous-namespace FlagStore helper in
// src/edge_op.cpp. It collects the indices in [0, n) that satisfy a
// predicate, then invokes a serial action f over them in ascending order.
//
// The predicate test may run in parallel (above the 1e5 threshold), but f
// always runs single-threaded: the edge-collapse / edge-swap operations it
// drives mutate shared mesh state and cannot be parallelized.
//
// Used by CollapseShortEdges, CollapseColinearEdges, and SwapDegenerates.
type flagStore struct {
	s []int
}

// run flags i in [0, n) where pred(i) is true, then calls f(i) for each
// flagged i in ascending order. Mirrors FlagStore::run: parallel pred test
// above 1e5, serial below.
func (fs *flagStore) run(n int, pred func(i int) bool, f func(i int)) {
	if n > 100000 {
		fs.runPar(n, pred, f)
	} else {
		fs.runSeq(n, pred, f)
	}
}

// runSeq mirrors FlagStore::run_seq: collect flagged indices in order, then
// process them in order.
func (fs *flagStore) runSeq(n int, pred func(i int) bool, f func(i int)) {
	fs.s = fs.s[:0]
	for i := 0; i < n; i++ {
		if pred(i) {
			fs.s = append(fs.s, i)
		}
	}
	for _, i := range fs.s {
		f(i)
	}
	fs.s = fs.s[:0]
}

// runPar mirrors FlagStore::run_par: test pred in parallel into per-worker
// buckets (tbb::combinable), concatenate, stable-sort to restore ascending
// order, then call f serially. The Go parallel package has no combinable, so
// the range is chunked across workers — the same pattern the DedupeEdges
// parallel branch uses to mirror a tbb::combinable.
func (fs *flagStore) runPar(n int, pred func(i int) bool, f func(i int)) {
	workers := runtime.GOMAXPROCS(0)
	if workers > n {
		workers = n
	}
	chunk := (n + workers - 1) / workers
	locals := make([][]int, workers)
	var wg sync.WaitGroup
	for w := 0; w < workers; w++ {
		lo := w * chunk
		hi := lo + chunk
		if hi > n {
			hi = n
		}
		if lo >= hi {
			continue
		}
		wg.Add(1)
		go func(w, lo, hi int) {
			defer wg.Done()
			var local []int
			for i := lo; i < hi; i++ {
				if pred(i) {
					local = append(local, i)
				}
			}
			locals[w] = local
		}(w, lo, hi)
	}
	wg.Wait()

	var result []int
	for _, l := range locals {
		result = append(result, l...)
	}
	// C++ stable_sorts the combined buckets (default threshold 1e4); the
	// chunked concatenation above is already ascending, but mirror it.
	parallel.StableSort(parallel.AutoPolicy(len(result), 10000), result, func(a, b int) bool { return a < b })
	for _, i := range result {
		f(i)
	}
}

// triRefSameFace mirrors C++ TriRef::SameFace (src/shared.h:305): two
// triangles belong to the same face iff their meshID, coplanarID, and
// faceID all match.
func triRefSameFace(a, b bridge.TriRef) bool {
	return a.MeshID == b.MeshID && a.CoplanarID == b.CoplanarID && a.FaceID == b.FaceID
}

// nanVec3 returns vec3(NAN) — the marker C++ writes into vertPos_ for a
// removed vertex.
func nanVec3() geom.Vec3 {
	n := math.NaN()
	return geom.Vec3{X: n, Y: n, Z: n}
}

// collapseTri is the Go port of Manifold::Impl::CollapseTri
// (src/edge_op.cpp:451). Removes the triangle whose three halfedges are
// triEdge, re-pairing the two surviving neighbor edges and marking all
// three halfedges removed.
func (s *dedupeState) collapseTri(triEdge [3]int) {
	if s.pair(triEdge[1]) == -1 {
		return
	}
	pair1 := int(s.pair(triEdge[1]))
	pair2 := int(s.pair(triEdge[2]))
	s.pairUp(pair1, pair2)
	for i := 0; i < 3; i++ {
		// halfedge_.Set(triEdge[i], -1, -1, Prop): start/end -> -1, prop kept.
		s.setStart(triEdge[i], -1)
		s.setEnd(triEdge[i], -1)
	}
}

// removeIfFolded is the Go port of Manifold::Impl::RemoveIfFolded
// (src/edge_op.cpp:461). If the two triangles sharing `edge` have folded
// onto each other (a degenerate bowtie), it NaNs the collapsed verts and
// removes both triangles, re-pairing their outer neighbors.
func (s *dedupeState) removeIfFolded(edge int) {
	tri0edge := triOf(edge)
	tri1edge := triOf(int(s.pair(edge)))
	if s.pair(tri0edge[1]) == -1 {
		return
	}
	if s.start(tri0edge[2]) == s.start(tri1edge[2]) {
		if int(s.pair(tri0edge[1])) == tri1edge[2] {
			if int(s.pair(tri0edge[2])) == tri1edge[1] {
				for i := 0; i < 3; i++ {
					s.verts[s.start(tri0edge[i])] = nanVec3()
				}
			} else {
				s.verts[s.start(tri0edge[1])] = nanVec3()
			}
		} else {
			if int(s.pair(tri0edge[2])) == tri1edge[1] {
				s.verts[s.start(tri1edge[1])] = nanVec3()
			}
		}
		s.pairUp(int(s.pair(tri0edge[1])), int(s.pair(tri1edge[2])))
		s.pairUp(int(s.pair(tri0edge[2])), int(s.pair(tri1edge[1])))
		for i := 0; i < 3; i++ {
			// halfedge_.Set(*, -1, -1, -1): start/end/prop all -> -1.
			s.setStart(tri0edge[i], -1)
			s.setEnd(tri0edge[i], -1)
			s.setProp(tri0edge[i], -1)
			s.setStart(tri1edge[i], -1)
			s.setEnd(tri1edge[i], -1)
			s.setProp(tri1edge[i], -1)
		}
	}
}

// formLoop is the Go port of Manifold::Impl::FormLoop (src/edge_op.cpp:433).
// Splits the mesh topologically by duplicating the two endpoints of
// `current` into fresh verts and re-stitching the loop. Note C++ grows only
// vertPos_ here (not vertNormal_); CalculateVertNormals runs afterward.
func (s *dedupeState) formLoop(current, end int) {
	startVert := len(s.verts)
	s.verts = append(s.verts, s.verts[s.start(current)])
	endVert := len(s.verts)
	s.verts = append(s.verts, s.verts[s.end(current)])

	oldMatch := int(s.pair(current))
	newMatch := int(s.pair(end))

	s.updateVert(int32(startVert), oldMatch, newMatch)
	s.updateVert(int32(endVert), end, current)

	s.pairUp(current, newMatch)
	s.pairUp(end, oldMatch)

	s.removeIfFolded(end)
}

// collapseEdge is the Go port of Manifold::Impl::CollapseEdge
// (src/edge_op.cpp:492). Collapses `edge` by removing its startVert and
// merging it into endVert, returning false if the collapse is rejected
// (would invert a triangle, cross a face boundary non-colinearly, etc.).
// May split the mesh via formLoop if the collapse would create a
// 4-manifold edge. `edges` is a reused scratch buffer (C++ Vec<int>&).
func (s *dedupeState) collapseEdge(edge int, edges *[]int, tol float64, firstNewVert int) bool {
	triRef := s.triRefs
	if tol < 0 {
		tol = s.epsilon
	}

	pair := int(s.pair(edge))
	if pair < 0 {
		return false
	}

	tri0edge := triOf(edge)
	tri1edge := triOf(pair)
	startVert := int(s.start(tri0edge[0]))
	endVert := int(s.start(tri0edge[1]))

	pNew := s.verts[endVert]
	pOld := s.verts[startVert]
	delta := pNew.Sub(pOld)
	maxLen := tol * tol
	if endVert >= firstNewVert {
		maxLen = s.epsilon * s.epsilon
	}
	shortEdge := delta.Dot(delta) < maxLen

	// Orbit startVert
	start := int(s.pair(tri1edge[1]))
	current := tri1edge[2]
	if !shortEdge {
		current = start
		refCheck := triRef[pair/3]
		pLast := s.verts[s.start(tri1edge[2])]
		for current != tri1edge[0] {
			current = nextHalfedge(current)
			pNext := s.verts[s.end(current)]
			tri := current / 3
			ref := triRef[tri]
			projection := geom.GetAxisAlignedProjection(s.faceNormals[tri])
			// Don't collapse if the edge is not redundant (may have changed
			// due to neighbor collapses).
			if !triRefSameFace(ref, refCheck) {
				oldRef := refCheck
				refCheck = triRef[edge/3]
				if !triRefSameFace(ref, refCheck) {
					return false
				}
				if ref.MeshID != oldRef.MeshID || ref.FaceID != oldRef.FaceID ||
					s.faceNormals[pair/3].Dot(s.faceNormals[tri]) < -0.5 {
					// Restrict collapse to colinear edges when the edge
					// separates faces or is sharp.
					if geom.CCW(projection.MulVec3(pLast), projection.MulVec3(pOld), projection.MulVec3(pNew), tol) != 0 {
						return false
					}
				}
			}
			// Don't collapse if it would invert a triangle.
			if geom.CCW(projection.MulVec3(pNext), projection.MulVec3(pLast), projection.MulVec3(pNew), s.epsilon) < 0 {
				return false
			}
			pLast = pNext
			current = int(s.pair(current))
		}
	}

	// Orbit endVert
	{
		current := int(s.pair(tri0edge[1]))
		for current != tri1edge[2] {
			current = nextHalfedge(current)
			*edges = append(*edges, current)
			current = int(s.pair(current))
		}
	}

	// Remove startVert and replace with endVert.
	s.verts[startVert] = nanVec3()
	s.collapseTri(tri1edge)

	// Orbit startVert
	tri0 := edge / 3
	tri1 := pair / 3
	current = start
	for current != tri0edge[2] {
		current = nextHalfedge(current)

		if s.numProp > 0 {
			// Update the shifted triangles to the propVert of endVert.
			tri := current / 3
			if triRefSameFace(triRef[tri], triRef[tri0]) {
				s.setProp(current, s.prop(nextHalfedge(edge)))
			} else if triRefSameFace(triRef[tri], triRef[tri1]) {
				s.setProp(current, s.prop(pair))
			}
		}

		vert := int(s.end(current))
		next := int(s.pair(current))
		for i := 0; i < len(*edges); i++ {
			if vert == int(s.end((*edges)[i])) {
				s.formLoop((*edges)[i], current)
				start = next
				*edges = (*edges)[:i]
				break
			}
		}
		current = next
	}

	s.updateVert(int32(endVert), start, tri0edge[2])
	s.collapseTri(tri0edge)
	s.removeIfFolded(start)
	return true
}
