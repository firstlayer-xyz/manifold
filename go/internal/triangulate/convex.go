package triangulate

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// IsConvex is the Go port of IsConvex (src/polygon.cpp:172). It reports
// whether every contour turns strictly left (CCW) at each vertex within
// epsilon — the condition for the convex fast path. Zero-length edges come
// out NaN from Normalize (which won't trip the early return), but the same
// edge tested non-normalized trips det <= 0.
func IsConvex(polys PolygonsIdx, epsilon float64) bool {
	for _, poly := range polys {
		n := len(poly)
		firstEdge := poly[0].Pos.Sub(poly[n-1].Pos)
		lastEdge := firstEdge.Normalize()
		for v := 0; v < n; v++ {
			edge := firstEdge
			if v+1 < n {
				edge = poly[v+1].Pos.Sub(poly[v].Pos)
			}
			det := geom.Determinant2x2(lastEdge, edge)
			if det <= 0 || (math.Abs(det) < epsilon && lastEdge.Dot(edge) < 0) {
				return false
			}
			lastEdge = edge.Normalize()
		}
	}
	return true
}

// TriangulateConvex is the Go port of TriangulateConvex (src/polygon.cpp:195):
// triangulates convex polygons by an alternating (zig-zag) fan rather than a
// simple fan, to avoid high-degree vertices. The exact i/k alternation order
// is deterministic and must match C++ bit-for-bit.
func TriangulateConvex(polys PolygonsIdx) *HalfedgeTriangulation {
	numTri := 0
	for _, poly := range polys {
		numTri += len(poly) - 2
	}
	result := newHalfedgeTriangulation()
	result.AddContours(polys)
	result.ReserveTriangles(numTri)
	for _, poly := range polys {
		i := 0
		k := len(poly) - 1
		right := true
		for i+1 < k {
			j := k - 1
			if right {
				j = i + 1
			}
			result.AddTriangle(poly[i].Idx, poly[j].Idx, poly[k].Idx)
			if right {
				i = j
			} else {
				k = j
			}
			right = !right
		}
	}
	return result
}

// TriangulateIdxHalfedges is the Go port of TriangulateIdxHalfedges
// (src/polygon.cpp:931): convex fast path, else the EarClip ear-clipper.
// EarClip is not yet ported (increments 5-7), so the concave branch returns
// (nil, false) and callers fall back to the reference triangulator.
func TriangulateIdxHalfedges(polys PolygonsIdx, epsilon float64, allowConvex bool) (*HalfedgeTriangulation, bool) {
	if allowConvex && IsConvex(polys, epsilon) {
		result := TriangulateConvex(polys)
		// The convex path leaves epsilon unresolved (matches C++: the -1 ->
		// bBox.Scale()*kPrecision resolution happens only inside EarClip).
		result.Epsilon = epsilon
		result.Finalize()
		return result, true
	}
	ec := newEarClip(polys, epsilon)
	result, ok := ec.triangulate()
	if !ok {
		// Has holes — keyholing (CutKeyhole) not yet ported (increment 7).
		return nil, false
	}
	result.Epsilon = ec.epsilon
	result.Finalize()
	return result, true
}

// TriangulateIdx returns the triangle index triples for indexed polygons,
// or (nil, false) when the concave EarClip path is needed (not yet ported).
func TriangulateIdx(polys PolygonsIdx, epsilon float64, allowConvex bool) ([][3]int, bool) {
	result, ok := TriangulateIdxHalfedges(polys, epsilon, allowConvex)
	if !ok {
		return nil, false
	}
	return result.Triangles(), true
}

// Triangulate is the Go port of Triangulate (src/polygon.cpp:996): assign a
// single running index to every vertex across all contours in order, then
// triangulate. Returns (triangles, handled); handled=false means the input
// is non-convex and the not-yet-ported EarClip path is required.
func Triangulate(polygons [][]geom.Vec2, epsilon float64, allowConvex bool) ([][3]int, bool) {
	idx := 0
	polys := make(PolygonsIdx, 0, len(polygons))
	for _, poly := range polygons {
		simple := make(SimplePolygonIdx, 0, len(poly))
		for _, p := range poly {
			simple = append(simple, PolyVert{Pos: p, Idx: idx})
			idx++
		}
		polys = append(polys, simple)
	}
	return TriangulateIdx(polys, epsilon, allowConvex)
}
