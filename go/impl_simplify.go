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
		// halfedge_.Set(idx, startVert, pairedHalfedge, propVert)
		// (shared.h:210): Set(triEdge[i], -1, -1, Prop) -> startVert and
		// pairedHalfedge become -1; propVert is kept (no-op).
		s.setStart(triEdge[i], -1)
		s.setPair(triEdge[i], -1)
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
			// halfedge_.Set(idx, startVert, pairedHalfedge, propVert):
			// Set(*, -1, -1, -1) -> startVert/pairedHalfedge/propVert all -1.
			s.setStart(tri0edge[i], -1)
			s.setPair(tri0edge[i], -1)
			s.setProp(tri0edge[i], -1)
			s.setStart(tri1edge[i], -1)
			s.setPair(tri1edge[i], -1)
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

// is01Longest mirrors Is01Longest (src/edge_op.cpp:33): true iff the edge
// v0->v1 is strictly the longest of the triangle's three edges.
func is01Longest(v0, v1, v2 geom.Vec2) bool {
	e0 := v1.Sub(v0)
	e1 := v2.Sub(v1)
	e2 := v0.Sub(v2)
	l0 := e0.Dot(e0)
	l1 := e1.Dot(e1)
	l2 := e2.Dot(e2)
	return l0 > l1 && l0 > l2
}

// recursiveEdgeSwap is the Go port of Manifold::Impl::RecursiveEdgeSwap
// (src/edge_op.cpp:608). Swaps the long edge of a degenerate triangle into
// its neighbor, then pushes the edges that may have newly become swappable
// onto edgeSwapStack for the caller to drain. visited[] is tagged per
// top-level flag to break infinite recursion; edges is a scratch buffer.
func (s *dedupeState) recursiveEdgeSwap(edge int, tag *int, visited []int, edgeSwapStack *[]int, edges *[]int) {
	triRef := s.triRefs

	if edge < 0 {
		return
	}
	pair := int(s.pair(edge))
	if pair < 0 {
		return
	}

	// avoid infinite recursion
	if visited[edge] == *tag && visited[pair] == *tag {
		return
	}

	tri0edge := triOf(edge)
	tri1edge := triOf(pair)

	projection := geom.GetAxisAlignedProjection(s.faceNormals[edge/3])
	var v [4]geom.Vec2
	for i := 0; i < 3; i++ {
		v[i] = projection.MulVec3(s.verts[s.start(tri0edge[i])])
	}
	// Only operate on the long edge of a degenerate triangle.
	if geom.CCW(v[0], v[1], v[2], s.tolerance) > 0 || !is01Longest(v[0], v[1], v[2]) {
		return
	}

	// Switch to neighbor's projection.
	projection = geom.GetAxisAlignedProjection(s.faceNormals[pair/3])
	for i := 0; i < 3; i++ {
		v[i] = projection.MulVec3(s.verts[s.start(tri0edge[i])])
	}
	v[3] = projection.MulVec3(s.verts[s.start(tri1edge[2])])

	swapEdge := func() {
		// The 0-verts are swapped to the opposite 2-verts.
		v0 := s.start(tri0edge[2])
		v1 := s.start(tri1edge[2])
		s.setStart(tri0edge[0], v1)
		s.setEnd(tri0edge[2], v1)
		s.setStart(tri1edge[0], v0)
		s.setEnd(tri1edge[2], v0)
		s.pairUp(tri0edge[0], int(s.pair(tri1edge[2])))
		s.pairUp(tri1edge[0], int(s.pair(tri0edge[2])))
		s.pairUp(tri0edge[2], tri1edge[2])
		// Both triangles are now subsets of the neighboring triangle.
		tri0 := tri0edge[0] / 3
		tri1 := tri1edge[0] / 3
		s.faceNormals[tri0] = s.faceNormals[tri1]
		triRef[tri0] = triRef[tri1]
		l01 := v[1].Sub(v[0]).Length()
		l02 := v[2].Sub(v[0]).Length()
		// a = std::max(0, std::min(1, l02/l01)); replicate C++ min/max NaN
		// semantics (NaN -> 1) so a degenerate l01 matches the C++ result.
		a := 1.0
		if ratio := l02 / l01; ratio < 1.0 {
			a = ratio
		}
		if !(0.0 < a) {
			a = 0.0
		}
		// Update properties if applicable.
		if len(s.properties) > 0 {
			s.setProp(tri0edge[1], s.prop(tri1edge[0]))
			s.setProp(tri0edge[0], s.prop(tri1edge[2]))
			s.setProp(tri0edge[2], s.prop(tri1edge[2]))
			numProp := s.numProp
			newProp := len(s.properties) / numProp
			propIdx0 := int(s.prop(tri1edge[0]))
			propIdx1 := int(s.prop(tri1edge[1]))
			for p := 0; p < numProp; p++ {
				s.properties = append(s.properties,
					a*s.properties[numProp*propIdx0+p]+(1-a)*s.properties[numProp*propIdx1+p])
			}
			s.setProp(tri1edge[0], int32(newProp))
			s.setProp(tri0edge[2], int32(newProp))
		}

		// if the new edge already exists, duplicate the verts and split.
		current := int(s.pair(tri1edge[0]))
		endVert := s.end(tri1edge[1])
		for current != tri0edge[1] {
			current = nextHalfedge(current)
			if s.end(current) == endVert {
				s.formLoop(tri0edge[2], current)
				s.removeIfFolded(tri0edge[2])
				return
			}
			current = int(s.pair(current))
		}
	}

	// Only operate if the other triangles are not degenerate.
	if geom.CCW(v[1], v[0], v[3], s.tolerance) <= 0 {
		if !is01Longest(v[1], v[0], v[3]) {
			return
		}
		// Two facing, long-edge degenerates can swap.
		swapEdge()
		e23 := v[3].Sub(v[2])
		if e23.Dot(e23) < s.tolerance*s.tolerance {
			*tag++
			s.collapseEdge(tri0edge[2], edges, -1, 0)
			*edges = (*edges)[:0]
		} else {
			visited[edge] = *tag
			visited[pair] = *tag
			for _, e := range []int{tri1edge[1], tri1edge[0], tri0edge[1], tri0edge[0]} {
				*edgeSwapStack = append(*edgeSwapStack, e)
			}
		}
		return
	} else if geom.CCW(v[0], v[3], v[2], s.tolerance) <= 0 ||
		geom.CCW(v[1], v[2], v[3], s.tolerance) <= 0 {
		return
	}
	// Normal path
	swapEdge()
	visited[edge] = *tag
	visited[pair] = *tag
	for _, e := range []int{int(s.pair(tri1edge[0])), int(s.pair(tri0edge[1]))} {
		*edgeSwapStack = append(*edgeSwapStack, e)
	}
}

// collapseShortEdges is the Go port of Manifold::Impl::CollapseShortEdges
// (src/edge_op.cpp:164). Flags edges below the collapse threshold and
// collapses each via collapseEdge. With firstNewVert == 0 (non-Boolean)
// the threshold is epsilon_; a Boolean pass sets firstNewVert and allows
// collapsing up to tolerance_ for edges touching new verts.
func (s *dedupeState) collapseShortEdges(firstNewVert int) {
	var fs flagStore
	nbEdges := s.numHalfedge()
	var scratch []int
	tol := s.epsilon
	if firstNewVert != 0 {
		tol = s.tolerance
	}
	shortEdge := func(edge int) bool {
		pair := int(s.pair(edge))
		if pair < 0 {
			return false
		}
		start := int(s.start(edge))
		end := int(s.end(edge))
		if start < firstNewVert && end < firstNewVert {
			return false
		}
		delta := s.verts[end].Sub(s.verts[start])
		lenSq := delta.Dot(delta)
		// end < firstNewVert (old vert) -> tol²; else (new vert) -> epsilon².
		maxLen := s.epsilon * s.epsilon
		if end < firstNewVert {
			maxLen = tol * tol
		}
		return lenSq < maxLen
	}
	fs.run(nbEdges, shortEdge, func(i int) {
		s.collapseEdge(i, &scratch, tol, firstNewVert)
		scratch = scratch[:0]
	})
}

// collapseColinearEdges is the Go port of Manifold::Impl::CollapseColinearEdges
// (src/edge_op.cpp:210). Repeatedly collapses edges whose startVert is
// surrounded by only two original faces (a redundant colinear vert),
// looping until a pass collapses nothing. "Colinear" is a GLOBAL property
// read via TriRef.SameFace (coplanarID, set upstream), not a local
// geometric test — which is what keeps it free of error stacking. Unlike
// CollapseShortEdges, the per-edge collapse always uses the 2-arg form
// (tol=-1 -> epsilon, firstNewVert=0); the outer firstNewVert only gates
// the predicate.
func (s *dedupeState) collapseColinearEdges(firstNewVert int) {
	var fs flagStore
	nbEdges := s.numHalfedge()
	var scratch []int
	for {
		numFlagged := 0
		colinearEdge := func(edge int) bool {
			pair := int(s.pair(edge))
			if pair < 0 || int(s.start(edge)) < firstNewVert {
				return false
			}
			// Flag redundant edges — startVert surrounded by only two
			// original triangles.
			ref0 := s.triRefs[edge/3]
			current := nextHalfedge(pair)
			ref1 := s.triRefs[current/3]
			ref1Updated := !triRefSameFace(ref0, ref1)
			for current != edge {
				current = nextHalfedge(int(s.pair(current)))
				ref := s.triRefs[current/3]
				if !triRefSameFace(ref, ref0) && !triRefSameFace(ref, ref1) {
					if !ref1Updated {
						ref1 = ref
						ref1Updated = true
					} else {
						return false
					}
				}
			}
			return true
		}
		fs.run(nbEdges, colinearEdge, func(i int) {
			if s.collapseEdge(i, &scratch, -1, 0) {
				numFlagged++
			}
			scratch = scratch[:0]
		})
		if numFlagged == 0 {
			break
		}
	}
}

// swapDegenerates is the Go port of Manifold::Impl::SwapDegenerates
// (src/edge_op.cpp:265). Flags degenerate (sliver) triangles whose long
// edge can be swapped into a neighbor, then runs recursiveEdgeSwap on each
// flagged edge, draining the resulting edge-swap stack to fixpoint.
func (s *dedupeState) swapDegenerates(firstNewVert int) {
	var fs flagStore
	nbEdges := s.numHalfedge()
	var scratch []int

	swappableEdge := func(edge int) bool {
		pair := int(s.pair(edge))
		if pair < 0 {
			return false
		}
		triEdge := triOf(edge)
		pairTriEdge := triOf(pair)
		if int(s.start(triEdge[0])) < firstNewVert &&
			int(s.start(triEdge[1])) < firstNewVert &&
			int(s.start(triEdge[2])) < firstNewVert &&
			int(s.start(pairTriEdge[2])) < firstNewVert {
			return false
		}
		projection := geom.GetAxisAlignedProjection(s.faceNormals[edge/3])
		var v [3]geom.Vec2
		for i := 0; i < 3; i++ {
			v[i] = projection.MulVec3(s.verts[s.start(triEdge[i])])
		}
		if geom.CCW(v[0], v[1], v[2], s.tolerance) > 0 || !is01Longest(v[0], v[1], v[2]) {
			return false
		}
		// Switch to neighbor's projection.
		projection = geom.GetAxisAlignedProjection(s.faceNormals[pair/3])
		for i := 0; i < 3; i++ {
			v[i] = projection.MulVec3(s.verts[s.start(pairTriEdge[i])])
		}
		return geom.CCW(v[0], v[1], v[2], s.tolerance) > 0 || is01Longest(v[0], v[1], v[2])
	}

	edgeSwapStack := []int{}
	visited := make([]int, nbEdges)
	for i := range visited {
		visited[i] = -1
	}
	tag := 0
	fs.run(nbEdges, swappableEdge, func(i int) {
		tag++
		s.recursiveEdgeSwap(i, &tag, visited, &edgeSwapStack, &scratch)
		for len(edgeSwapStack) > 0 {
			last := edgeSwapStack[len(edgeSwapStack)-1]
			edgeSwapStack = edgeSwapStack[:len(edgeSwapStack)-1]
			s.recursiveEdgeSwap(last, &tag, visited, &edgeSwapStack, &scratch)
		}
	})
}

// RemoveDegenerates is the Go port of Manifold::Impl::RemoveDegenerates
// (src/edge_op.cpp:153): clean up topology, collapse short edges, swap
// degenerate slivers, then recompute vert normals (collapses move verts).
// firstNewVert restricts edits to newly-created verts (Boolean passes it;
// 0 otherwise). This is the gating step for the Manifold(MeshGL) ingest
// path.
func (mi *MutableImpl) RemoveDegenerates(firstNewVert int) {
	if len(mi.HalfedgeStarts()) == 0 {
		return
	}
	mi.CleanupTopology()
	s := newDedupeState(mi)
	s.collapseShortEdges(firstNewVert)
	s.swapDegenerates(firstNewVert)
	s.commit(mi)
	mi.CalculateVertNormals()
}

// SimplifyTopology is the Go port of Manifold::Impl::SimplifyTopology
// (src/edge_op.cpp:141): RemoveDegenerates plus an extra
// CollapseColinearEdges pass between the short-edge and swap passes.
// Used by the Boolean result cleanup and the Refine family. firstNewVert
// restricts edits to newly-created verts (Boolean passes it; 0 otherwise).
func (mi *MutableImpl) SimplifyTopology(firstNewVert int) {
	if len(mi.HalfedgeStarts()) == 0 {
		return
	}
	mi.CleanupTopology()
	s := newDedupeState(mi)
	s.collapseShortEdges(firstNewVert)
	s.collapseColinearEdges(firstNewVert)
	s.swapDegenerates(firstNewVert)
	s.commit(mi)
	mi.CalculateVertNormals()
}
