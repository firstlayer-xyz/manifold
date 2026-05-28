package manifold

import "github.com/firstlayer-xyz/manifold/go/internal/parallel"

// checkHalfedge is the Go port of the C++ CheckHalfedges functor in
// src/properties.cpp. Validates a single halfedge index against the
// surrounding halfedge_ array invariants:
//
//   - removed sentinel (start == end == pair == -1): valid.
//   - either of the same triangle's other halfedges has start == -1:
//     invalid (triangle in an inconsistent partial-removal state).
//   - pair == -1 (boundary): invalid (oriented closed mesh has no
//     boundaries).
//   - otherwise:
//     Pair(Pair(e)) == e   (reciprocity)
//     start != end         (no self-loops)
//     start == End(pair)   (pair shares vertices in reversed order)
//     end == Start(pair)
func checkHalfedge(edge int, starts, pairs []int32) bool {
	start := int(starts[edge])
	// End(e) = Start(NextHalfedge(e)).
	end := int(starts[nextHalfedge(edge)])
	pair := int(pairs[edge])
	if start == -1 && end == -1 && pair == -1 {
		return true
	}
	if int(starts[nextHalfedge(edge)]) == -1 ||
		int(starts[nextHalfedge(nextHalfedge(edge))]) == -1 {
		return false
	}
	if pair == -1 {
		return false
	}
	if int(pairs[pair]) != edge {
		return false
	}
	if start == end {
		return false
	}
	if start != int(starts[nextHalfedge(pair)]) {
		return false
	}
	if end != int(starts[pair]) {
		return false
	}
	return true
}

// IsManifold is the Go port of C++ Manifold::Impl::IsManifold
// (src/properties.cpp). True iff every halfedge passes checkHalfedge
// (and the count is a multiple of 3, since every triangle contributes
// three halfedges).
//
// C++ uses `all_of(countAt(0), countAt(halfedge_.size()),
// CheckHalfedges{halfedge_})` — the no-policy form, which defaults to
// autoPolicy with threshold 1e5. We mirror via parallel.AllOf.
func (i *Impl) IsManifold() bool {
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	n := len(starts)
	if n == 0 {
		return true
	}
	if n%3 != 0 {
		return false
	}
	return parallel.AllOf(parallel.AutoPolicy(n, 100000), n, func(e int) bool {
		return checkHalfedge(e, starts, pairs)
	})
}

// IsManifold mirror on MutableImpl (Go has no const-correctness so
// the read-only check is exposed on both facade types).
func (mi *MutableImpl) IsManifold() bool {
	starts := mi.HalfedgeStarts()
	pairs := mi.HalfedgePairs()
	n := len(starts)
	if n == 0 {
		return true
	}
	if n%3 != 0 {
		return false
	}
	return parallel.AllOf(parallel.AutoPolicy(n, 100000), n, func(e int) bool {
		return checkHalfedge(e, starts, pairs)
	})
}

// halfedgeTuple is the Go port of the C++ Halfedge struct (start,
// end, pair) used by Is2Manifold's sort+adjacency check. Ordered
// lexicographically by (startVert, endVert) matching C++ operator<.
type halfedgeTuple struct {
	start, end, pair int32
}

// Is2Manifold is the Go port of C++ Manifold::Impl::Is2Manifold
// (src/properties.cpp): returns true iff the mesh is a 2-manifold
// (no shared edge appears more than twice). Algorithm: assert
// IsManifold first, then stable-sort the halfedges and check that
// no two adjacent post-sort halfedges share the same (start, end)
// pair — duplicate (start, end) means the edge has > 2 incident
// triangles.
//
// Removed-sentinel halfedges (start == end == pair == -1) are
// exempt — they always pass the adjacency test by the C++ rule.
func (i *Impl) Is2Manifold() bool {
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	n := len(starts)
	if n == 0 {
		return true
	}
	if !i.IsManifold() {
		return false
	}
	// Build a sortable view of (start, end, pair). End(idx) =
	// Start(NextHalfedge(idx)).
	hes := make([]halfedgeTuple, n)
	parallel.ForEachN(parallel.AutoPolicy(n, 100000), n, func(idx int) {
		hes[idx] = halfedgeTuple{
			start: starts[idx],
			end:   starts[nextHalfedge(idx)],
			pair:  pairs[idx],
		}
	})
	parallel.StableSort(parallel.AutoPolicy(n, 10000), hes, func(a, b halfedgeTuple) bool {
		if a.start != b.start {
			return a.start < b.start
		}
		return a.end < b.end
	})
	// 2 * NumEdge() - 1 is the loop upper bound in C++ (so we read
	// hes[edge+1] safely). NumEdge = halfedge.size() / 2.
	limit := n - 1
	return parallel.AllOf(parallel.AutoPolicy(limit, 100000), limit, func(edge int) bool {
		h := hes[edge]
		if h.start == -1 && h.end == -1 && h.pair == -1 {
			return true
		}
		nxt := hes[edge+1]
		return h.start != nxt.start || h.end != nxt.end
	})
}
