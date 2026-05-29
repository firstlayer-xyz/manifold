package boolean

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// halfedges is a read-view of the Impl's halfedge arrays (Go port of the C++
// Halfedges accessor: Start/End/Pair) over the bridge-exposed slices. End(e) is
// the start of the next halfedge in the triangle (the manifold invariant), as
// elsewhere in the Go port.
type halfedges struct {
	starts []int32
	pairs  []int32
}

func (h halfedges) Start(e int) int { return int(h.starts[e]) }
func (h halfedges) End(e int) int   { return int(h.starts[nextHalfedge(e)]) }
func (h halfedges) Pair(e int) int  { return int(h.pairs[e]) }

// nextHalfedge is the Go port of NextHalfedge (src/shared.h): the next halfedge
// within the same triangle, via Next3 ((e/3)*3 + Next3(e%3)).
func nextHalfedge(e int) int { return e - e%3 + Next3(e%3) }

// intersect is the Go port of Intersect (boolean3.cpp:34): the (x, y, z_a, z_b)
// of the crossing of the xy-projected segments aL-aR and bL-bR, in the kernel's
// bespoke smaller-magnitude interpolation form. NOT geom.Vec4.Lerp — the float
// expression order here is load-bearing and transcribed byte-for-byte.
func intersect(aL, aR, bL, bR geom.Vec3) geom.Vec4 {
	dyL := bL.Y - aL.Y
	dyR := bR.Y - aR.Y
	// C++ DEBUG_ASSERT(dyL*dyR <= 0): no-intersection guard, debug-only.
	useL := math.Abs(dyL) < math.Abs(dyR)
	dx := aR.X - aL.X
	num := dyR
	if useL {
		num = dyL
	}
	lambda := num / (dyL - dyR)
	if !isFinite(lambda) {
		lambda = 0.0
	}
	var xyzz geom.Vec4
	xBase := aR.X
	if useL {
		xBase = aL.X
	}
	xyzz.X = lambda*dx + xBase

	aDy := aR.Y - aL.Y
	bDy := bR.Y - bL.Y
	useA := math.Abs(aDy) < math.Abs(bDy)
	dyTerm := bDy
	if useA {
		dyTerm = aDy
	}
	var yBase float64
	if useL {
		yBase = bL.Y
		if useA {
			yBase = aL.Y
		}
	} else {
		yBase = bR.Y
		if useA {
			yBase = aR.Y
		}
	}
	xyzz.Y = lambda*dyTerm + yBase

	zaBase := aR.Z
	if useL {
		zaBase = aL.Z
	}
	xyzz.Z = lambda*(aR.Z-aL.Z) + zaBase

	zbBase := bR.Z
	if useL {
		zbBase = bL.Z
	}
	xyzz.W = lambda*(bR.Z-bL.Z) + zbBase
	return xyzz
}

// faceEdge is the Go port of the FaceEdge struct (boolean3.cpp:55): an edge of a
// triangle, oriented forward (start < end) with isForward recording whether the
// canonical halfedge was the forward one.
type faceEdge struct {
	edge      int
	start     int
	end       int
	isForward bool
}

// loadFaceEdges is the Go port of LoadFaceEdges (boolean3.cpp:62): the three
// edges of triangle tri, each canonicalized to start < end (taking the paired
// halfedge when the stored one runs backward).
func loadFaceEdges(h halfedges, tri int, edge *[3]faceEdge) {
	for i := 0; i < 3; i++ {
		halfedge := 3*tri + i
		start := h.Start(halfedge)
		end := h.Start(3*tri + Next3(i))
		if start < end {
			edge[i] = faceEdge{edge: halfedge, start: start, end: end, isForward: true}
		} else {
			edge[i] = faceEdge{edge: h.Pair(halfedge), start: end, end: start, isForward: false}
		}
	}
}
