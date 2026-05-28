package manifold

import (
	"runtime"
	"testing"
)

// TestPublicValidity_Primitives exercises the public Manifold.Is*
// methods across the primitive constructors. Each primitive must be
// a manifold, a 2-manifold, convex, finite, and not self-intersecting.
func TestPublicValidity_Primitives(t *testing.T) {
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
			if !tc.m.IsManifold() {
				t.Errorf("%s: IsManifold = false", tc.name)
			}
			if !tc.m.Is2Manifold() {
				t.Errorf("%s: Is2Manifold = false", tc.name)
			}
			if !tc.m.IsConvex() {
				t.Errorf("%s: IsConvex = false", tc.name)
			}
			if !tc.m.IsFinite() {
				t.Errorf("%s: IsFinite = false", tc.name)
			}
			if tc.m.IsSelfIntersecting() {
				t.Errorf("%s: IsSelfIntersecting = true", tc.name)
			}
		})
	}
}

// TestPublicValidity_BooleanResult: cube - smaller offset cube is
// still a valid 2-manifold, non-self-intersecting, finite, but NOT
// convex (it has a concave corner).
func TestPublicValidity_BooleanResult(t *testing.T) {
	outer := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(outer)
	inner := Cube(Vec3{X: 1, Y: 1, Z: 1}, false).Translate(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer runtime.KeepAlive(inner)
	cut := outer.Difference(inner)
	defer runtime.KeepAlive(cut)

	if !cut.IsManifold() {
		t.Error("Difference result should be IsManifold")
	}
	if !cut.Is2Manifold() {
		t.Error("Difference result should be Is2Manifold")
	}
	if cut.IsConvex() {
		t.Error("cube-minus-corner-cube should NOT be convex")
	}
	if !cut.IsFinite() {
		t.Error("Difference result should be IsFinite")
	}
	if cut.IsSelfIntersecting() {
		t.Error("Difference result should NOT be self-intersecting")
	}
}
