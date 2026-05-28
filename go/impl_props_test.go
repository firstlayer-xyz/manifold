package manifold

import (
	"math"
	"runtime"
	"testing"
)

// TestIsFinite_Primitives: every freshly-constructed primitive has
// finite vertex positions. Mirrors the C++ debug-assert pattern that
// IsFinite() holds after construction.
func TestIsFinite_Primitives(t *testing.T) {
	cases := []struct {
		name string
		m    *Manifold
	}{
		{"Tetrahedron", Tetrahedron()},
		{"Cube", Cube(Vec3{X: 1, Y: 1, Z: 1}, false)},
		{"Sphere", Sphere(1.0, 24)},
		{"Cylinder", Cylinder(2.0, 0.5, 0.5, 16, false)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			defer runtime.KeepAlive(tc.m)
			impl := getImpl(tc.m)
			defer impl.Delete()
			if !impl.IsFinite() {
				t.Errorf("%s should be IsFinite", tc.name)
			}
		})
	}
}

// TestIsFinite_DetectsNaN injects a NaN vert into a MutableImpl copy
// (bypassing the public API which would reject the warp) and verifies
// IsFinite reports false.
func TestIsFinite_DetectsNaN(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	src := getImpl(m)
	defer src.Delete()
	mi := src.Copy()
	defer mi.Delete()
	mi.Verts()[0].X = math.NaN()
	if mi.IsFinite() {
		t.Error("IsFinite should be false after injecting a NaN vert")
	}
}

// TestIsConvex_Primitives: Cube/Tetrahedron/Sphere are convex;
// Cylinder is convex. Use Hull which should always yield convex.
func TestIsConvex_Primitives(t *testing.T) {
	cases := []struct {
		name string
		m    *Manifold
		want bool
	}{
		{"Tetrahedron", Tetrahedron(), true},
		{"Cube", Cube(Vec3{X: 1, Y: 1, Z: 1}, false), true},
		{"Sphere", Sphere(1.0, 24), true},
		{"Cylinder", Cylinder(2.0, 0.5, 0.5, 16, false), true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			defer runtime.KeepAlive(tc.m)
			impl := getImpl(tc.m)
			defer impl.Delete()
			if got := impl.IsConvex(); got != tc.want {
				t.Errorf("%s IsConvex: got %v, want %v", tc.name, got, tc.want)
			}
		})
	}
}

// TestIsSelfIntersecting_Primitives: every freshly-constructed
// primitive must be self-intersection-free.
func TestIsSelfIntersecting_Primitives(t *testing.T) {
	cases := []struct {
		name string
		m    *Manifold
	}{
		{"Tetrahedron", Tetrahedron()},
		{"Cube", Cube(Vec3{X: 1, Y: 1, Z: 1}, false)},
		{"Sphere", Sphere(1.0, 24)},
		{"Cylinder", Cylinder(2.0, 0.5, 0.5, 16, false)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			defer runtime.KeepAlive(tc.m)
			impl := getImpl(tc.m)
			defer impl.Delete()
			if impl.IsSelfIntersecting() {
				t.Errorf("%s should NOT be self-intersecting", tc.name)
			}
		})
	}
}

// TestIsConvex_Concave subtracts a smaller cube from the corner of a
// larger one — the result has concave edges where the cavity meets
// the outer surface, so IsConvex must be false.
func TestIsConvex_Concave(t *testing.T) {
	outer := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(outer)
	inner := Cube(Vec3{X: 1, Y: 1, Z: 1}, false).Translate(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer runtime.KeepAlive(inner)
	concave := outer.Difference(inner)
	defer runtime.KeepAlive(concave)
	impl := getImpl(concave)
	defer impl.Delete()
	if impl.IsConvex() {
		t.Error("a cube-minus-corner-cube should NOT be convex")
	}
}
