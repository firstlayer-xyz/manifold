// Package triangulate is the Go port of src/polygon.cpp — manifold's
// ear-clipping polygon triangulator. It is self-contained 2D computational
// geometry with no dependency on the Boolean/CSG engine.
package triangulate

import "github.com/firstlayer-xyz/manifold/go/internal/geom"

// kPrecision mirrors src/utils.h kPrecision — the relative tolerance used to
// resolve a negative epsilon against the polygon's bounding-box scale.
const kPrecision = 1e-12

// PolyVert is a polygon vertex carrying its index into the source vertex
// list. Mirrors C++ PolyVert (include/manifold/polygon.h:27).
type PolyVert struct {
	Pos geom.Vec2
	Idx int
}

// SimplePolygonIdx is a single contour, wound CCW, with indices. First and
// last point are implicitly connected. Mirrors C++ SimplePolygonIdx.
type SimplePolygonIdx = []PolyVert

// PolygonsIdx is a set of indexed contours, which may include holes and any
// number of separate polygons. Mirrors C++ PolygonsIdx.
type PolygonsIdx = []SimplePolygonIdx

// Halfedge is the triangulation-output halfedge. Mirrors src/shared.h
// Halfedge; PropVert is unused by triangulation and stays -1.
type Halfedge struct {
	StartVert, EndVert, PairedHalfedge, PropVert int
}

// HalfedgeTriangulation accumulates the triangulation as a paired halfedge
// mesh. Verbatim port of src/polygon_internal.h:26-112. edge2halfedge is a
// transient pairing index (C++ std::unordered_map; iteration order is never
// observed, so a plain Go map is faithful) that Finalize clears.
type HalfedgeTriangulation struct {
	Halfedges     []Halfedge
	ContourEnd    int
	Epsilon       float64
	edge2halfedge map[uint64][]int
}

func newHalfedgeTriangulation() *HalfedgeTriangulation {
	return &HalfedgeTriangulation{Epsilon: -1, edge2halfedge: map[uint64][]int{}}
}

// AddContours registers the exterior contour halfedges (each stored opposite
// the filled interior, AddHalfedge(end, start)) and records contourEnd.
func (t *HalfedgeTriangulation) AddContours(polys PolygonsIdx) {
	for _, poly := range polys {
		n := len(poly)
		for i := 0; i < n; i++ {
			start := poly[i].Idx
			next := 0
			if i+1 < n {
				next = i + 1
			}
			end := poly[next].Idx
			t.addHalfedge(end, start)
		}
	}
	t.ContourEnd = len(t.Halfedges)
}

// ReserveTriangles is a capacity hint; Go slices grow automatically, so the
// C++ reserve is a no-op for behavior (kept for API parity / one allocation).
func (t *HalfedgeTriangulation) ReserveTriangles(numTri int) {
	want := t.ContourEnd + 3*numTri
	if cap(t.Halfedges) < want {
		grown := make([]Halfedge, len(t.Halfedges), want)
		copy(grown, t.Halfedges)
		t.Halfedges = grown
	}
}

// AddTriangle appends the three halfedges of a triangle (first->second->third).
func (t *HalfedgeTriangulation) AddTriangle(first, second, third int) {
	t.addHalfedge(first, second)
	t.addHalfedge(second, third)
	t.addHalfedge(third, first)
}

// NumTri returns the number of triangles added (excluding contour halfedges).
func (t *HalfedgeTriangulation) NumTri() int {
	return (len(t.Halfedges) - t.ContourEnd) / 3
}

// Triangles reads the triangle halfedges in groups of 3 and returns their
// start-vert index triples.
func (t *HalfedgeTriangulation) Triangles() [][3]int {
	tris := make([][3]int, 0, t.NumTri())
	for edge := t.ContourEnd; edge < len(t.Halfedges); edge += 3 {
		tris = append(tris, [3]int{
			t.Halfedges[edge].StartVert,
			t.Halfedges[edge+1].StartVert,
			t.Halfedges[edge+2].StartVert,
		})
	}
	return tris
}

// Finalize clears the transient pairing index. (The C++ MANIFOLD_DEBUG
// reciprocity assertions are intentionally omitted from the release port.)
func (t *HalfedgeTriangulation) Finalize() { t.edge2halfedge = nil }

func edgeKey(start, end int) uint64 {
	return uint64(uint32(start))<<32 | uint64(uint32(end))
}

// addHalfedge appends a halfedge (start->end), pairing it with a previously
// added reverse halfedge (end->start) if one is waiting.
func (t *HalfedgeTriangulation) addHalfedge(start, end int) {
	halfedge := len(t.Halfedges)
	data := Halfedge{StartVert: start, EndVert: end, PairedHalfedge: -1, PropVert: -1}
	revKey := edgeKey(end, start)
	rev := t.edge2halfedge[revKey]
	if len(rev) > 0 {
		data.PairedHalfedge = rev[len(rev)-1]
		t.Halfedges[data.PairedHalfedge].PairedHalfedge = halfedge
		rev = rev[:len(rev)-1]
		if len(rev) == 0 {
			delete(t.edge2halfedge, revKey)
		} else {
			t.edge2halfedge[revKey] = rev
		}
	} else {
		k := edgeKey(start, end)
		t.edge2halfedge[k] = append(t.edge2halfedge[k], halfedge)
	}
	t.Halfedges = append(t.Halfedges, data)
}
