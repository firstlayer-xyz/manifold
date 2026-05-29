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
func TriangulateIdxHalfedges(polys PolygonsIdx, epsilon float64, allowConvex bool) *HalfedgeTriangulation {
	result := newHalfedgeTriangulation()
	updatedEpsilon := epsilon
	if allowConvex && IsConvex(polys, epsilon) { // fast path
		result = TriangulateConvex(polys)
	} else {
		ec := newEarClip(polys, epsilon)
		result = ec.triangulate()
		updatedEpsilon = ec.epsilon
	}
	result.Epsilon = updatedEpsilon
	result.Finalize()
	return result
}

// TriangulateIdx is the Go port of TriangulateIdx (src/polygon.cpp:972):
// the triangle index triples for indexed polygons.
func TriangulateIdx(polys PolygonsIdx, epsilon float64, allowConvex bool) [][3]int {
	return TriangulateIdxHalfedges(polys, epsilon, allowConvex).Triangles()
}

// Triangulate is the Go port of Triangulate (src/polygon.cpp:996): assign a
// single running index to every vertex across all contours in order, then
// triangulate.
func Triangulate(polygons [][]geom.Vec2, epsilon float64, allowConvex bool) [][3]int {
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
