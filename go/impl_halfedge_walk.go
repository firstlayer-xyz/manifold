package manifold

// Halfedge-walking primitives — ports of the inline helpers in
// src/shared.h and src/impl.h that the algorithm files depend on
// (vertex-fan traversal, next-within-tri).

// nextHalfedge mirrors NextHalfedge in src/shared.h:
//
//	current += current % 3 == 2 ? -2 : 1
//
// Walks to the next halfedge of the same triangle.
func nextHalfedge(current int) int {
	if current%3 == 2 {
		return current - 2
	}
	return current + 1
}

// prevHalfedge mirrors PrevHalfedge in src/shared.h:38:
//
//	current += current % 3 == 0 ? 2 : -1
//
// Walks to the previous halfedge of the same triangle.
func prevHalfedge(current int) int {
	if current%3 == 0 {
		return current + 2
	}
	return current - 1
}

// triOf returns the three halfedge indices of the triangle containing
// edge, in order {edge, next, next-next}. Mirrors C++ TriOf
// (src/edge_op.cpp:25).
func triOf(edge int) [3]int {
	a := edge
	b := nextHalfedge(a)
	c := nextHalfedge(b)
	return [3]int{a, b, c}
}

// forVert is the Go port of C++ Manifold::Impl::ForVert(int, F) from
// src/impl.h. Starting at halfedge h, walks current = NextHalfedge(
// Pair(current)) around the vertex fan, calling fn(current) at each
// step, terminating when current returns to h. The fan visits every
// halfedge incident to the start vertex of h, with h itself as the
// last visit.
//
// Caller passes the impl's halfedge.paired_ slice as `pairs` to keep
// this helper free of facade-type plumbing.
func forVert(start int, pairs []int32, fn func(edge int)) {
	current := start
	for {
		current = nextHalfedge(int(pairs[current]))
		fn(current)
		if current == start {
			return
		}
	}
}

// forVertTransform is the Go port of the two-arg C++ template
// `void ForVert<T>(int halfedge, transform, binaryOp)` from
// src/impl.h. At each step it maps the current edge via `transform`
// and calls `binaryOp(current, here, &next)`, where `here` is the
// transformed value at the previous step. The binary op may mutate
// `next` via the pointer; the mutated value becomes `here` for the
// following iteration. This is the same contract as the C++
// `T& next` reference parameter.
func forVertTransform[T any](
	start int,
	pairs []int32,
	transform func(edge int) T,
	binaryOp func(edge int, here T, next *T),
) {
	here := transform(start)
	current := start
	for {
		nextEdge := nextHalfedge(int(pairs[current]))
		next := transform(nextEdge)
		binaryOp(current, here, &next)
		here = next
		current = nextEdge
		if current == start {
			return
		}
	}
}
