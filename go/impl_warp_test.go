package manifold

import (
	"math"
	"runtime"
	"testing"
)

// TestWarp_NonFiniteFlagsError: a callback that produces a NaN
// vertex must drive the resulting Manifold's status to
// NonFiniteVertex, matching the C++ contract that IsFinite() == false
// after warpFunc returns triggers MakeEmpty(NonFiniteVertex).
//
// We NaN exactly one vert (the first +X +Y +Z corner) so that the
// surviving 7 corners still span a finite bbox — otherwise
// CalculateBBox itself collapses the impl with status NoError before
// the IsFinite check fires. Matches C++ which has the same edge-case
// behavior (covered separately by all-NaN scenarios).
func TestWarp_NonFiniteFlagsError(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	tagged := false
	bad := m.Warp(func(v *Vec3) {
		if !tagged && v.X > 0 && v.Y > 0 && v.Z > 0 {
			v.X = math.NaN()
			tagged = true
		}
	})
	defer runtime.KeepAlive(bad)
	if got, want := bad.Status(), NonFiniteVertex; got != want {
		t.Errorf("Status after non-finite Warp: got %v, want %v", got, want)
	}
}

// TestWarp_PreservesIdentity: a no-op Warp must yield a manifold
// equivalent (Volume, SurfaceArea, NumVert) to the input. This
// indirectly verifies that the finalize tail (CalculateBBox →
// SetEpsilon → SortGeometry → SetNormalsAndCoplanar) is wired
// correctly.
func TestWarp_PreservesIdentity(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	id := m.Warp(func(v *Vec3) {})
	defer runtime.KeepAlive(id)
	if !floatClose(id.Volume(), m.Volume(), 1e-12, 1e-12) {
		t.Errorf("identity Warp Volume: got %v, want %v", id.Volume(), m.Volume())
	}
	if !floatClose(id.SurfaceArea(), m.SurfaceArea(), 1e-12, 1e-12) {
		t.Errorf("identity Warp SurfaceArea: got %v, want %v",
			id.SurfaceArea(), m.SurfaceArea())
	}
	if got, want := id.NumVert(), m.NumVert(); got != want {
		t.Errorf("identity Warp NumVert: got %d, want %d", got, want)
	}
}

// TestWarpBatch_NonFiniteFlagsError mirrors TestWarp_NonFiniteFlagsError
// for the batch entry point.
func TestWarpBatch_NonFiniteFlagsError(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	bad := m.WarpBatch(func(verts []Vec3) {
		// NaN the first vertex.
		verts[0].X = math.NaN()
	})
	defer runtime.KeepAlive(bad)
	if got, want := bad.Status(), NonFiniteVertex; got != want {
		t.Errorf("Status after non-finite WarpBatch: got %v, want %v", got, want)
	}
}
