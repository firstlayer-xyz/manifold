package triangulate

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/stdcpp/multiset"
)

// vert is a node of the circularly-linked list representing the polygon(s)
// still to be triangulated. Mirrors EarClip::Vert (src/polygon.cpp:314).
//
// left/right are *vert pointers into earClip.polygon, a slice pre-allocated to
// its final capacity so it never reallocates — matching the C++ reliance on
// stable iterators into a reserve()'d std::vector<Vert>.
type vert struct {
	meshIdx       int
	cost          float64
	ear           multiset.Handle[*vert] // earsQueue_ handle; invalid == earsQueue_.end()
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
	holes *multiset.Multiset[*vert]
	// outers / simples: starts per positive-area contour / per simple polygon.
	outers  []*vert
	simples []*vert
	// hole2BBox maps each hole (by start vert) to its bounding box.
	hole2BBox map[*vert]geom.Rect
	// earsQueue is the priority queue of valid ears, ordered by cost (MinCost).
	earsQueue *multiset.Multiset[*vert]
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
		holes:     multiset.New[*vert](func(a, b *vert) bool { return a.pos.X > b.pos.X }),
		earsQueue: multiset.New[*vert](func(a, b *vert) bool { return a.cost < b.cost }),
		hole2BBox: map[*vert]geom.Rect{},
		result:    newHalfedgeTriangulation(),
		bBox:      geom.EmptyRect(),
		epsilon:   epsilon,
	}
	ec.starts = ec.initialize(polys)
	for i := range ec.polygon {
		ec.clipIfDegenerate(&ec.polygon[i])
	}
	for _, first := range ec.starts {
		ec.findStart(first)
	}
	return ec
}

// kBest is the lowest possible cost (clip-immediately), mirroring
// src/polygon.cpp:36 kBest = -infinity.
var kBest = math.Inf(-1)

// triangulate is the Go port of EarClip::Triangulate (src/polygon.cpp:255):
// key-hole every hole into an outer contour, then ear-clip each simple polygon.
func (ec *earClip) triangulate() *HalfedgeTriangulation {
	for _, start := range ec.holes.InOrder() {
		ec.cutKeyhole(start)
	}
	for _, start := range ec.simples {
		ec.triangulatePoly(start)
	}
	return ec.result
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

// findStart is the Go port of EarClip::FindStart (src/polygon.cpp:655): walk a
// contour, accumulate its signed area (Kahan/Neumaier) and bounding box, find
// the rightmost reflex vert, and classify the contour as a hole (negative
// area) or a simple/outer polygon (positive area).
func (ec *earClip) findStart(first *vert) {
	origin := first.pos
	start := first
	maxX := math.Inf(-1)
	bBox := geom.EmptyRect()
	area := 0.0
	areaCompensation := 0.0

	addPoint := func(v *vert) {
		bBox = bBox.Union(v.pos)
		area1 := geom.Determinant2x2(v.pos.Sub(origin), v.right.pos.Sub(origin))
		t1 := area + area1
		areaCompensation += (area - t1) + area1
		area = t1
		if v.pos.X > maxX && v.isReflex(ec.epsilon) {
			maxX = v.pos.X
			start = v
		}
	}

	if ec.loop(first, addPoint) == nil {
		// No polygon left — all ears were degenerate and already clipped.
		return
	}

	area += areaCompensation
	size := bBox.Size()
	minArea := ec.epsilon * math.Max(size.X, size.Y)

	if !math.IsInf(maxX, 0) && !math.IsNaN(maxX) && area < -minArea {
		ec.holes.Insert(start)
		ec.hole2BBox[start] = bBox
	} else {
		ec.simples = append(ec.simples, start)
		if area > minArea {
			ec.outers = append(ec.outers, start)
		}
	}
}

// cutKeyhole is the Go port of EarClip::CutKeyhole (src/polygon.cpp:703): find
// the outer-contour edge a hole's rightmost start connects to (a horizontal
// "keyhole" to the right), refine it, and bridge the two polygons.
func (ec *earClip) cutKeyhole(start *vert) {
	bBox := ec.hole2BBox[start]
	onTop := 0
	if start.pos.Y >= bBox.Max.Y-ec.epsilon {
		onTop = 1
	} else if start.pos.Y <= bBox.Min.Y+ec.epsilon {
		onTop = -1
	}
	var connector *vert // nil == polygon_.end()

	checkEdge := func(edge *vert) {
		x := edge.interpY2X(start.pos, onTop, ec.epsilon)
		if isFinite(x) && start.insideEdge(edge, ec.epsilon, true) &&
			(connector == nil ||
				geom.CCW(geom.Vec2{X: x, Y: start.pos.Y}, connector.pos, connector.right.pos, ec.epsilon) == 1 ||
				func() bool {
					if connector.pos.Y < edge.pos.Y {
						return edge.insideEdge(connector, ec.epsilon, false)
					}
					return !connector.insideEdge(edge, ec.epsilon, false)
				}()) {
			connector = edge
		}
	}

	for _, first := range ec.outers {
		ec.loop(first, checkEdge)
	}

	if connector == nil {
		// Hole did not find an outer contour; treat it as its own simple poly.
		ec.simples = append(ec.simples, start)
		return
	}

	connector = ec.findCloserBridge(start, connector)

	ec.joinPolygons(start, connector)
}

// findCloserBridge is the Go port of EarClip::FindCloserBridge
// (src/polygon.cpp:748): convert the initial keyhole guess into the final one
// by finding any reflex verts inside the triangle containing the best
// connection and the initial horizontal line.
func (ec *earClip) findCloserBridge(start, edge *vert) *vert {
	var connector *vert
	switch {
	case edge.pos.X < start.pos.X:
		connector = edge.right
	case edge.right.pos.X < start.pos.X:
		connector = edge
	case edge.right.pos.Y-start.pos.Y > start.pos.Y-edge.pos.Y:
		connector = edge
	default:
		connector = edge.right
	}
	if math.Abs(connector.pos.Y-start.pos.Y) <= ec.epsilon {
		return connector
	}
	above := -1.0
	if connector.pos.Y > start.pos.Y {
		above = 1
	}

	checkVert := func(v *vert) {
		inside := above * float64(geom.CCW(start.pos, v.pos, connector.pos, ec.epsilon))
		if v.pos.X > start.pos.X-ec.epsilon &&
			v.pos.Y*above > start.pos.Y*above-ec.epsilon &&
			(inside > 0 || (inside == 0 && v.pos.X < connector.pos.X &&
				v.pos.Y*above < connector.pos.Y*above)) &&
			v.insideEdge(edge, ec.epsilon, true) && v.isReflex(ec.epsilon) {
			connector = v
		}
	}

	for _, first := range ec.outers {
		ec.loop(first, checkVert)
	}

	return connector
}

// joinPolygons is the Go port of EarClip::JoinPolygons (src/polygon.cpp:783):
// create a keyhole between a hole's start vert and the connector vert of an
// outer polygon by duplicating both verts and reattaching the four links. The
// duplication can create degenerate ears, so they are clipped immediately.
func (ec *earClip) joinPolygons(start, connector *vert) {
	newStart := ec.pushVert(*start)
	newConnector := ec.pushVert(*connector)

	start.right.left = newStart
	connector.left.right = newConnector
	link(start, connector)
	link(newConnector, newStart)

	ec.clipIfDegenerate(start)
	ec.clipIfDegenerate(newStart)
	ec.clipIfDegenerate(connector)
	ec.clipIfDegenerate(newConnector)
}

// idxCollider is the Go port of EarClip::IdxCollider (src/polygon.cpp:306): the
// kd-tree points (each carrying an index back into itr) plus the verts they
// reference. BuildTwoDTree reorders points in place, so point.Idx is how
// EarCost recovers the original vert after the reorder.
type idxCollider struct {
	points []PolyVert
	itr    []*vert
}

// processEar is the Go port of EarClip::ProcessEar (src/polygon.cpp:802):
// recompute v's cost and update its position in earsQueue_ (remove + reinsert).
func (ec *earClip) processEar(v *vert, collider idxCollider) {
	if v.ear.Valid() {
		ec.earsQueue.Erase(v.ear)
		v.ear = multiset.Handle[*vert]{}
	}
	if v.isShort(ec.epsilon) {
		v.cost = kBest
		v.ear = ec.earsQueue.Insert(v)
	} else if v.isConvex(2 * ec.epsilon) {
		v.cost = v.earCost(ec.epsilon, collider)
		v.ear = ec.earsQueue.Insert(v)
	} else {
		v.cost = 1 // reflex; not inserted (marker only)
	}
}

// vertCollider is the Go port of EarClip::VertCollider (src/polygon.cpp:821):
// collect all un-clipped verts of the polygon into a kd-tree, each point
// expanded (in EarCost) by epsilon. Each ear uses this tree to quickly find the
// subset of verts to check for cost.
func (ec *earClip) vertCollider(start *vert) idxCollider {
	var itr []*vert
	var points []PolyVert
	ec.loop(start, func(v *vert) {
		points = append(points, PolyVert{Pos: v.pos, Idx: len(itr)})
		itr = append(itr, v)
	})
	buildTwoDTree(points)
	return idxCollider{points: points, itr: itr}
}

// triangulatePoly is the Go port of EarClip::TriangulatePoly
// (src/polygon.cpp:836): the main ear-clipping loop for one simple polygon.
func (ec *earClip) triangulatePoly(start *vert) {
	collider := ec.vertCollider(start)
	if len(collider.itr) == 0 {
		return
	}

	// A simple polygon always creates two fewer triangles than it has verts.
	numTri := -2
	ec.earsQueue.Clear()

	queueVert := func(v *vert) {
		ec.processEar(v, collider)
		numTri++
	}

	v := ec.loop(start, queueVert)
	if v == nil {
		return
	}

	for numTri > 0 {
		if ear, ok := ec.earsQueue.Begin(); ok {
			v = ear.Value()
			ec.earsQueue.Erase(ear)
		}
		// else: no ear found (geometrically invalid) — reuse the carried-over
		// backup v to ensure manifoldness.
		ec.clipEar(v)
		numTri--
		ec.processEar(v.left, collider)
		ec.processEar(v.right, collider)
		v = v.right // backup vert for the empty-queue case
	}
}
