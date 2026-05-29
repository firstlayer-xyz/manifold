package manifold

import (
	"math"
	"slices"
	"sort"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// regularPolygon returns a single CCW-wound regular n-gon of radius r.
func regularPolygon(n int, r float64) [][]geom.Vec2 {
	poly := make([]geom.Vec2, n)
	for i := 0; i < n; i++ {
		a := 2 * math.Pi * float64(i) / float64(n) // increasing angle -> CCW
		poly[i] = geom.Vec2{X: r * math.Cos(a), Y: r * math.Sin(a)}
	}
	return [][]geom.Vec2{poly}
}

// TestTriangulateConvex_VsCpp checks the native convex fast path against the
// C++ bridge. C++ also routes convex input through TriangulateConvex, so the
// zig-zag fan is deterministic and the exact triangle index triples must match.
func TestTriangulateConvex_VsCpp(t *testing.T) {
	cases := []struct {
		name  string
		polys [][]geom.Vec2
	}{
		{"triangle", regularPolygon(3, 1)},
		{"square", regularPolygon(4, 1)},
		{"pentagon", regularPolygon(5, 1)},
		{"decagon", regularPolygon(10, 2)},
		{"circle64", regularPolygon(64, 1)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := triangulateNative(tc.polys, -1)
			want := bridge.Triangulate(tc.polys, -1)
			// Convex input routes through TriangulateConvex in both Go and C++,
			// so the zig-zag fan is deterministic and the triples match exactly.
			if !slices.Equal(got, want) {
				t.Fatalf("triangles differ:\n native=%v\n bridge=%v", got, want)
			}
		})
	}
}

// triangleSet canonicalizes a flat triangle-index list into a sorted set of
// winding-preserving triangles (each rotated so its smallest index is first).
// Two triangulations with the same set are identical up to emission order.
func triangleSet(flat []int32) [][3]int32 {
	canon := func(a, b, c int32) [3]int32 {
		if a <= b && a <= c {
			return [3]int32{a, b, c}
		}
		if b <= a && b <= c {
			return [3]int32{b, c, a}
		}
		return [3]int32{c, a, b}
	}
	tris := make([][3]int32, 0, len(flat)/3)
	for i := 0; i+2 < len(flat); i += 3 {
		tris = append(tris, canon(flat[i], flat[i+1], flat[i+2]))
	}
	sort.Slice(tris, func(i, j int) bool {
		if tris[i][0] != tris[j][0] {
			return tris[i][0] < tris[j][0]
		}
		if tris[i][1] != tris[j][1] {
			return tris[i][1] < tris[j][1]
		}
		return tris[i][2] < tris[j][2]
	})
	return tris
}

// sameTriangulation reports whether two flat triangle lists are the same
// triangulation (same triangle set, emission order aside).
func sameTriangulation(a, b []int32) bool {
	sa, sb := triangleSet(a), triangleSet(b)
	if len(sa) != len(sb) {
		return false
	}
	for i := range sa {
		if sa[i] != sb[i] {
			return false
		}
	}
	return true
}

// checkValidTriangulation asserts that flat is a valid triangulation of polys:
// every triangle is CCW (non-negative signed area) and the triangle areas sum
// to the polygon's area (no gaps/overlaps). Triangle indices reference the
// running-index vertex order the idx wrapper assigns.
func checkValidTriangulation(t *testing.T, polys [][]geom.Vec2, flat []int32) {
	t.Helper()
	var verts []geom.Vec2
	for _, p := range polys {
		verts = append(verts, p...)
	}
	polyArea2 := 0.0
	for _, p := range polys {
		for i := range p {
			j := (i + 1) % len(p)
			polyArea2 += geom.Determinant2x2(p[i], p[j])
		}
	}
	tol := 1e-9 * math.Abs(polyArea2)
	sum2 := 0.0
	for i := 0; i+2 < len(flat); i += 3 {
		a, b, c := verts[flat[i]], verts[flat[i+1]], verts[flat[i+2]]
		area2 := geom.Determinant2x2(b.Sub(a), c.Sub(a))
		if area2 < -tol {
			t.Errorf("non-CCW triangle (%d,%d,%d): 2*area=%v", flat[i], flat[i+1], flat[i+2], area2)
		}
		sum2 += area2
	}
	if math.Abs(sum2-polyArea2) > tol {
		t.Errorf("triangulation area %v != polygon area %v (gaps/overlaps)", sum2/2, polyArea2/2)
	}
}

// starPolygon returns a single CCW concave star with `points` spikes.
func starPolygon(points int, rOuter, rInner float64) [][]geom.Vec2 {
	n := points * 2
	poly := make([]geom.Vec2, n)
	for i := 0; i < n; i++ {
		a := math.Pi * float64(i) / float64(points) // CCW
		r := rOuter
		if i%2 == 1 {
			r = rInner
		}
		poly[i] = geom.Vec2{X: r * math.Cos(a), Y: r * math.Sin(a)}
	}
	return [][]geom.Vec2{poly}
}

// TestTriangulateConcave_VsCpp exercises the native EarClip ear-clipper on
// simple concave polygons (no holes). The output must be a valid triangulation
// (correct count, tiles the polygon, all-CCW) with the same triangle count as
// the C++ bridge. An exact triangle-set match is asserted where it holds, but
// is not required: EarCost uses normalize/dot/determinant whose last-ULP
// differences between Go and C++ can flip the clip order of near-equal-cost
// ears (symmetric stars), yielding a different — but equally valid —
// triangulation. C++'s own TestPoly checks only the triangle count, and the
// project compares geometry semantically rather than bit-identically.
func TestTriangulateConcave_VsCpp(t *testing.T) {
	lShape := [][]geom.Vec2{{
		{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1},
		{X: 1, Y: 1}, {X: 1, Y: 2}, {X: 0, Y: 2},
	}}
	plus := [][]geom.Vec2{{
		{X: 1, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1}, {X: 3, Y: 1}, {X: 3, Y: 2},
		{X: 2, Y: 2}, {X: 2, Y: 3}, {X: 1, Y: 3}, {X: 1, Y: 2}, {X: 0, Y: 2},
		{X: 0, Y: 1}, {X: 1, Y: 1},
	}}
	cases := []struct {
		name  string
		polys [][]geom.Vec2
	}{
		{"L_shape", lShape},
		{"plus", plus},
		{"star5", starPolygon(5, 2, 0.8)},
		{"star8", starPolygon(8, 3, 1.2)},
		{"star20", starPolygon(20, 5, 4.0)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := triangulateNative(tc.polys, -1)
			want := bridge.Triangulate(tc.polys, -1)
			// The Go output must be a genuine triangulation of the polygon.
			checkValidTriangulation(t, tc.polys, got)
			// ... with the same triangle count as C++.
			if len(got) != len(want) {
				t.Errorf("triangle count %d != bridge %d", len(got)/3, len(want)/3)
			}
			// Exact set-match where float allows; otherwise both are valid.
			if !sameTriangulation(got, want) {
				t.Logf("%s: triangulation differs from C++ as a set (both valid; float-tied ear costs)", tc.name)
			}
		})
	}
}

// TestTriangulateHoles_VsCpp exercises the key-holing path (CutKeyhole /
// FindCloserBridge / JoinPolygons): polygons with one or more CW holes inside a
// CCW outer contour. The native EarClip must bridge each hole into an outer and
// produce a valid triangulation of the region between them, with the same
// triangle count as the C++ bridge.
func TestTriangulateHoles_VsCpp(t *testing.T) {
	// CCW outer, CW hole(s).
	square1Hole := [][]geom.Vec2{
		{{X: 0, Y: 0}, {X: 6, Y: 0}, {X: 6, Y: 6}, {X: 0, Y: 6}},
		{{X: 2, Y: 2}, {X: 2, Y: 4}, {X: 4, Y: 4}, {X: 4, Y: 2}},
	}
	square2Holes := [][]geom.Vec2{
		{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 6}, {X: 0, Y: 6}},
		{{X: 1, Y: 1}, {X: 1, Y: 3}, {X: 3, Y: 3}, {X: 3, Y: 1}},
		{{X: 6, Y: 2}, {X: 6, Y: 4}, {X: 8, Y: 4}, {X: 8, Y: 2}},
	}
	// Hole shifted to the right edge — stresses the rightward keyhole search.
	offCenterHole := [][]geom.Vec2{
		{{X: 0, Y: 0}, {X: 8, Y: 0}, {X: 8, Y: 4}, {X: 0, Y: 4}},
		{{X: 5, Y: 1}, {X: 5, Y: 3}, {X: 7, Y: 3}, {X: 7, Y: 1}},
	}
	// Triangular hole (odd vert count) inside a pentagon outer.
	triHole := [][]geom.Vec2{
		{{X: 0, Y: 0}, {X: 6, Y: 0}, {X: 7, Y: 4}, {X: 3, Y: 7}, {X: -1, Y: 4}},
		{{X: 2, Y: 2}, {X: 2, Y: 4}, {X: 4, Y: 3}},
	}
	cases := []struct {
		name  string
		polys [][]geom.Vec2
	}{
		{"square_1hole", square1Hole},
		{"square_2holes", square2Holes},
		{"offcenter_hole", offCenterHole},
		{"tri_hole", triHole},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := triangulateNative(tc.polys, -1)
			want := bridge.Triangulate(tc.polys, -1)
			checkValidTriangulation(t, tc.polys, got)
			if len(got) != len(want) {
				t.Errorf("triangle count %d != bridge %d", len(got)/3, len(want)/3)
			}
			if !sameTriangulation(got, want) {
				t.Logf("%s: triangulation differs from C++ as a set (both valid; float-tied ear costs)", tc.name)
			}
		})
	}
}
