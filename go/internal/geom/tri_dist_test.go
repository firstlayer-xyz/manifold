package geom

import (
	"math"
	"testing"
)

// TestDistanceTriangleTriangleSquared_Disjoint: two parallel
// triangles offset by Z=1 have squared distance 1.
func TestDistanceTriangleTriangleSquared_Disjoint(t *testing.T) {
	p := [3]Vec3{
		{X: 0, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
	}
	q := [3]Vec3{
		{X: 0, Y: 0, Z: 1},
		{X: 1, Y: 0, Z: 1},
		{X: 0, Y: 1, Z: 1},
	}
	got := DistanceTriangleTriangleSquared(p, q)
	if math.Abs(got-1.0) > 1e-12 {
		t.Errorf("disjoint parallel tris dd² = %v, want 1", got)
	}
}

// TestDistanceTriangleTriangleSquared_Touching: two coplanar
// triangles sharing an edge have distance 0.
func TestDistanceTriangleTriangleSquared_Touching(t *testing.T) {
	p := [3]Vec3{
		{X: 0, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
	}
	q := [3]Vec3{
		{X: 1, Y: 0, Z: 0},
		{X: 1, Y: 1, Z: 0},
		{X: 0, Y: 1, Z: 0},
	}
	got := DistanceTriangleTriangleSquared(p, q)
	if got > 1e-12 {
		t.Errorf("touching tris dd² = %v, want ~0", got)
	}
}

// TestDistanceTriangleTriangleSquared_Intersecting: two triangles
// that cross through each other have distance 0.
func TestDistanceTriangleTriangleSquared_Intersecting(t *testing.T) {
	p := [3]Vec3{
		{X: -1, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 1},
	}
	q := [3]Vec3{
		{X: 0, Y: -1, Z: 0.5},
		{X: 0, Y: 1, Z: 0.5},
		{X: 0, Y: 0, Z: -0.5},
	}
	got := DistanceTriangleTriangleSquared(p, q)
	if got > 1e-12 {
		t.Errorf("intersecting tris dd² = %v, want 0", got)
	}
}

// TestEdgeEdgeDist_Crossing: two perpendicular edges crossing at
// the origin have distance 0.
func TestEdgeEdgeDist_Crossing(t *testing.T) {
	x, y := EdgeEdgeDist(
		Vec3{X: -1}, Vec3{X: 2}, // segment 1: from (-1,0,0) to (1,0,0)
		Vec3{Y: -1}, Vec3{Y: 2}, // segment 2: from (0,-1,0) to (0,1,0)
	)
	d := x.Sub(y)
	if d.Dot(d) > 1e-12 {
		t.Errorf("crossing segments closest dist = %v, want 0 (x=%+v, y=%+v)",
			math.Sqrt(d.Dot(d)), x, y)
	}
}
