package triangulate

import "github.com/firstlayer-xyz/manifold/go/internal/geom"

// vert is a node of the circularly-linked list representing the polygon(s)
// still to be triangulated. Mirrors EarClip::Vert (src/polygon.cpp:314).
//
// left/right are *vert pointers into earClip.polygon, a slice pre-allocated to
// its final capacity so it never reallocates — matching the C++ reliance on
// stable iterators into a reserve()'d std::vector<Vert>.
type vert struct {
	meshIdx       int
	cost          float64
	ear           msHandle[*vert] // earsQueue_ handle; invalid == earsQueue_.end()
	pos, rightDir geom.Vec2
	left, right   *vert
}

// isShort mirrors Vert::IsShort (polygon.cpp:323): true when the edge to the
// right neighbor is shorter than half of epsilon (conservative, to avoid CW
// triangles exceeding epsilon from rounding).
func (v *vert) isShort(epsilon float64) bool {
	edge := v.right.pos.Sub(v.pos)
	return edge.Dot(edge)*4 < epsilon*epsilon
}

// earClip is the Go port of class EarClip (src/polygon.cpp:233). This file
// covers the ring construction + lifecycle; the predicates, cost function,
// clip loop (TriangulatePoly), FindStart, and keyholing land in later
// increments.
type earClip struct {
	// polygon is the flat backing store of all verts. Pre-allocated to
	// numVert + 2*numContours and never grown past cap, so *vert stays stable.
	polygon []vert
	// holes is the set of right-most starts (one per negative-area contour),
	// ordered by pos.x descending (MaxX).
	holes *orderedMultiset[*vert]
	// outers / simples: starts per positive-area contour / per simple polygon.
	outers  []*vert
	simples []*vert
	// hole2BBox maps each hole (by start vert) to its bounding box.
	hole2BBox map[*vert]geom.Rect
	// earsQueue is the priority queue of valid ears, ordered by cost (MinCost).
	earsQueue *orderedMultiset[*vert]
	// result is the output triangulation as halfedges.
	result *HalfedgeTriangulation
	// bBox is the bounding box of the entire polygon set.
	bBox geom.Rect
	// epsilon is the working tolerance (resolved from -1 in Initialize).
	epsilon float64
	// starts holds the per-contour arbitrary start verts from Initialize,
	// consumed by FindStart (later increment).
	starts []*vert
}

// newEarClip is the Go port of the EarClip ctor (src/polygon.cpp:235). It
// reserves the backing store, builds the rings (Initialize), and clips
// degenerate ears. (FindStart over starts is added with the predicates in a
// later increment.)
func newEarClip(polys PolygonsIdx, epsilon float64) *earClip {
	numVert := 0
	for _, poly := range polys {
		numVert += len(poly)
	}
	ec := &earClip{
		polygon:   make([]vert, 0, numVert+2*len(polys)),
		holes:     newOrderedMultiset[*vert](func(a, b *vert) bool { return a.pos.X > b.pos.X }),
		earsQueue: newOrderedMultiset[*vert](func(a, b *vert) bool { return a.cost < b.cost }),
		hole2BBox: map[*vert]geom.Rect{},
		result:    newHalfedgeTriangulation(),
		bBox:      geom.EmptyRect(),
		epsilon:   epsilon,
	}
	ec.starts = ec.initialize(polys)
	for i := range ec.polygon {
		ec.clipIfDegenerate(&ec.polygon[i])
	}
	return ec
}

// pushVert appends a vert and returns a stable pointer to it. Panics if the
// backing slice would reallocate — the reserve()-equivalent invariant that
// keeps every *vert valid (Initialize pushes numVert; JoinPolygons pushes up
// to 2 per hole, both within the reserved cap).
func (ec *earClip) pushVert(v vert) *vert {
	if len(ec.polygon) == cap(ec.polygon) {
		panic("earClip: polygon backing slice would reallocate (cap underestimated)")
	}
	ec.polygon = append(ec.polygon, v)
	return &ec.polygon[len(ec.polygon)-1]
}

// link is the Go port of EarClip::Link (polygon.cpp:531). It (with
// JoinPolygons) is the only writer of the circular-list pointers, and also
// caches the normalized edge direction.
func link(left, right *vert) {
	left.right = right
	right.left = left
	left.rightDir = right.pos.Sub(left.pos).SafeNormalize()
}

// clipped is the Go port of EarClip::Clipped (polygon.cpp:539): a vert is
// clipped when its right neighbor no longer links back to it.
func clipped(v *vert) bool { return v.right.left != v }

// loop is the Go port of EarClip::Loop (polygon.cpp:543): apply fn to each
// un-clipped vert and return an un-clipped vert (nil on collapse to 2).
func (ec *earClip) loop(first *vert, fn func(*vert)) *vert {
	v := first
	for {
		if clipped(v) {
			// Re-anchor first to an un-clipped vert to avoid infinite looping.
			first = v.right.left
			if !clipped(first) {
				v = first
				if v.right == v.left {
					return nil
				}
				fn(v)
			}
		} else {
			if v.right == v.left {
				return nil
			}
			fn(v)
		}
		v = v.right
		if v == first {
			break
		}
	}
	return v
}

// clipEar is the Go port of EarClip::ClipEar (polygon.cpp:571): unlink the ear
// and emit its triangle, filtering topological degenerates (verts sharing a
// mesh_idx, which can arise from hole-vert duplication).
func (ec *earClip) clipEar(ear *vert) {
	link(ear.left, ear.right)
	if ear.left.meshIdx != ear.meshIdx &&
		ear.meshIdx != ear.right.meshIdx &&
		ear.right.meshIdx != ear.left.meshIdx {
		ec.result.AddTriangle(ear.left.meshIdx, ear.meshIdx, ear.right.meshIdx)
	}
}

// clipIfDegenerate is the Go port of EarClip::ClipIfDegenerate
// (polygon.cpp:588): recursively clip ears that would form a degenerate
// triangle (short edge, or colinear with the neighbors pointing the same way).
func (ec *earClip) clipIfDegenerate(ear *vert) {
	if clipped(ear) {
		return
	}
	if ear.left == ear.right {
		return
	}
	if ear.isShort(ec.epsilon) ||
		(geom.CCW(ear.left.pos, ear.pos, ear.right.pos, ec.epsilon) == 0 &&
			ear.left.pos.Sub(ear.pos).Dot(ear.right.pos.Sub(ear.pos)) > 0) {
		ec.clipEar(ear)
		ec.clipIfDegenerate(ear.left)
		ec.clipIfDegenerate(ear.right)
	}
}

// initialize is the Go port of EarClip::Initialize (polygon.cpp:605): build
// the circular lists, accumulate the bounding box, resolve epsilon, and
// register the contour halfedges. Returns the per-contour (arbitrary) starts.
func (ec *earClip) initialize(polys PolygonsIdx) []*vert {
	var starts []*vert
	for _, poly := range polys {
		first := ec.pushVert(vert{meshIdx: poly[0].Idx, pos: poly[0].Pos})
		ec.bBox = ec.bBox.Union(first.pos)
		last := first
		// Not the real rightmost start yet — just an arbitrary identifying vert.
		starts = append(starts, first)
		for k := 1; k < len(poly); k++ {
			ec.bBox = ec.bBox.Union(poly[k].Pos)
			next := ec.pushVert(vert{meshIdx: poly[k].Idx, pos: poly[k].Pos})
			link(last, next)
			last = next
		}
		link(last, first)
	}
	if ec.epsilon < 0 {
		ec.epsilon = ec.bBox.Scale() * kPrecision
	}
	ec.result.AddContours(polys)
	// Slightly more than enough: each hole can cause two extra triangles.
	ec.result.ReserveTriangles(len(ec.polygon) + 2*len(starts))
	return starts
}
