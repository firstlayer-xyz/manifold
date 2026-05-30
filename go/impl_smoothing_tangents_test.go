package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// TestCircularTangent validates the pure Bezier math of circularTangent
// (the float-order-sensitive heart of tangent generation) against two
// hand-derivable cases.
func TestCircularTangent(t *testing.T) {
	approx := func(a, b geom.Vec4) bool {
		const eps = 1e-15
		return math.Abs(a.X-b.X) <= eps && math.Abs(a.Y-b.Y) <= eps &&
			math.Abs(a.Z-b.Z) <= eps && math.Abs(a.W-b.W) <= eps
	}
	cases := []struct {
		name             string
		tangent, edgeVec geom.Vec3
		want             geom.Vec4
	}{
		{
			// Tangent aligned with the edge: weight = max(0.5, 1) = 1, so the
			// circular tangent is the straight-line cubic control offset = 1/3
			// of the edge vector, weight 1.
			name:    "aligned",
			tangent: geom.Vec3{X: 1},
			edgeVec: geom.Vec3{X: 1},
			want:    geom.Vec4{X: 1.0 / 3.0, W: 1},
		},
		{
			// Tangent perpendicular to the edge: dot = 0, weight clamps to 0.5.
			// bz3 = {0, 1/3, 0, 2/3}; de-homogenized = {0, 0.5, 0, 2/3}.
			name:    "perpendicular",
			tangent: geom.Vec3{Y: 1},
			edgeVec: geom.Vec3{X: 1},
			want:    geom.Vec4{Y: 0.5, W: 2.0 / 3.0},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := circularTangent(tc.tangent, tc.edgeVec)
			if !approx(got, tc.want) {
				t.Fatalf("circularTangent(%v, %v) = %+v, want %+v", tc.tangent, tc.edgeVec, got, tc.want)
			}
		})
	}
}

// TestCreateTangentsIdx_VsCpp differential-tests the native CreateTangents
// against the C++ bridge CreateTangentsIdx on the same Impl copies. Tangents
// involve acos/sin/cos (stdlib vs C++ musl), so they're compared semantically
// (tight tolerance), not bit-exact. Exercises both the smooth path (sphere,
// minSharpAngle 0) and the discontinuity/fixed-tangent path (cube, sharp edges).
func TestCreateTangentsIdx_VsCpp(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
	}{
		{"sphere_smooth", func() *Manifold { return Sphere(1, 32).CalculateNormals(0, 0) }},
		{"cube_sharp", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, true).CalculateNormals(0, 60) }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			defer runtime.KeepAlive(m)
			view := getImpl(m)
			defer view.Delete()

			a := view.Copy()
			defer a.Delete()
			a.CreateTangents(0)
			got := append([]float64(nil), a.HalfedgeTangents()...)

			b := view.Copy()
			defer b.Delete()
			// Bridge reference reads/writes the bridge handle; marshal around it so
			// the tangents reload into native storage for HalfedgeTangents().
			b.runBridgeAlgo(func() { b.h.CreateTangentsIdx(0) })
			want := append([]float64(nil), b.HalfedgeTangents()...)

			if len(got) != len(want) {
				t.Fatalf("tangent count: native %d != bridge %d", len(got), len(want))
			}
			maxDiff := 0.0
			maxAt := 0
			for i := range got {
				d := math.Abs(got[i] - want[i])
				if d > maxDiff {
					maxDiff, maxAt = d, i
				}
			}
			t.Logf("max abs tangent diff = %g at index %d (native %v, bridge %v)",
				maxDiff, maxAt, got[maxAt], want[maxAt])
			// Tangents use acos/sin/cos (Go stdlib vs C++ musl); near-zero
			// components accumulate ~1e-9 noise. A structural divergence would
			// be orders larger, so 1e-7 cleanly separates noise from bugs.
			if maxDiff > 1e-7 {
				t.Fatalf("max tangent diff %g exceeds tolerance (index %d)", maxDiff, maxAt)
			}
		})
	}
}

// TestCreateTangentsFromSmoothness_VsCpp differential-tests the native
// createTangentsFromSmoothness against the bridge, feeding both the same
// sharpened-edge list (from SharpenEdges). Compared semantically (acos/sin/cos
// stdlib vs musl). Cube exercises many sharp edges (size-2 continuous + uniform
// vert cases); sphere exercises mostly-smooth with a few sharpened edges.
func TestCreateTangentsFromSmoothness_VsCpp(t *testing.T) {
	cases := []struct {
		name  string
		mk    func() *Manifold
		angle float64
	}{
		{"cube", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, true) }, 60},
		{"sphere", func() *Manifold { return Sphere(1, 16) }, 70},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			defer runtime.KeepAlive(m)
			view := getImpl(m)
			defer view.Delete()
			sharp := view.SharpenEdges(tc.angle, 0)

			a := view.Copy()
			defer a.Delete()
			a.CreateTangentsFromSmoothness(sharp)
			got := append([]float64(nil), a.HalfedgeTangents()...)

			b := view.Copy()
			defer b.Delete()
			b.runBridgeAlgo(func() { b.h.CreateTangentsFromSmoothness(sharp) })
			want := append([]float64(nil), b.HalfedgeTangents()...)

			if len(got) != len(want) {
				t.Fatalf("tangent count: native %d != bridge %d", len(got), len(want))
			}
			maxDiff := 0.0
			maxAt := 0
			for i := range got {
				d := math.Abs(got[i] - want[i])
				if d > maxDiff {
					maxDiff, maxAt = d, i
				}
			}
			t.Logf("max abs tangent diff = %g at index %d (native %v, bridge %v)",
				maxDiff, maxAt, got[maxAt], want[maxAt])
			// Tangents use acos/sin/cos (Go stdlib vs C++ musl); near-zero
			// components accumulate ~1e-9 noise. A structural divergence would
			// be orders larger, so 1e-7 cleanly separates noise from bugs.
			if maxDiff > 1e-7 {
				t.Fatalf("max tangent diff %g exceeds tolerance (index %d)", maxDiff, maxAt)
			}
		})
	}
}

// TestSmoothByNormals_NativeRefine exercises the now-native CreateTangents
// through the public SmoothByNormals API: CalculateNormals(0) populates slot 0,
// SmoothByNormals(0) derives tangents from them, and Refine consumes the
// tangents to add verts. Confirms the production facade wiring works end to end.
func TestSmoothByNormals_NativeRefine(t *testing.T) {
	m := Sphere(1, 16).CalculateNormals(0, 0)
	defer runtime.KeepAlive(m)
	sm := m.SmoothByNormals(0)
	defer runtime.KeepAlive(sm)
	if sm.IsEmpty() {
		t.Fatal("SmoothByNormals returned empty")
	}
	if sm.Status() != NoError {
		t.Fatalf("SmoothByNormals Status: %v", sm.Status())
	}
	refined := sm.Refine(4)
	defer runtime.KeepAlive(refined)
	if refined.NumVert() <= sm.NumVert() {
		t.Errorf("Refine after SmoothByNormals should add verts: before=%d after=%d",
			sm.NumVert(), refined.NumVert())
	}
}
