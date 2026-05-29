package manifold

import (
	"math"
	"slices"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/triangulate"
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
			// Confirm the native convex path is actually taken (not fallback).
			if _, ok := triangulate.Triangulate(tc.polys, -1, true); !ok {
				t.Fatalf("expected convex fast path for %s", tc.name)
			}
			got := triangulateNative(tc.polys, -1)
			want := bridge.Triangulate(tc.polys, -1)
			if !slices.Equal(got, want) {
				t.Fatalf("triangles differ:\n native=%v\n bridge=%v", got, want)
			}
		})
	}
}

// TestTriangulateConcave_Fallback verifies IsConvex rejects a concave polygon
// (so it does NOT take the convex path and emit wrong triangles), and that the
// bridge fallback matches. The L-shape is concave at the inner corner.
func TestTriangulateConcave_Fallback(t *testing.T) {
	lShape := [][]geom.Vec2{{
		{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1},
		{X: 1, Y: 1}, {X: 1, Y: 2}, {X: 0, Y: 2},
	}}
	if _, ok := triangulate.Triangulate(lShape, -1, true); ok {
		t.Fatalf("L-shape is concave; convex fast path must NOT handle it")
	}
	got := triangulateNative(lShape, -1)
	want := bridge.Triangulate(lShape, -1)
	if !slices.Equal(got, want) {
		t.Fatalf("concave fallback differs:\n native=%v\n bridge=%v", got, want)
	}
}
