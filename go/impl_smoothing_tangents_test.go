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

// TestCreateTangentsIdx_VsCpp differential-tests the native createTangentsIdx
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
			a.createTangentsIdx(0)
			got := append([]float64(nil), a.HalfedgeTangents()...)

			b := view.Copy()
			defer b.Delete()
			b.h.CreateTangentsIdx(0)
			want := append([]float64(nil), b.HalfedgeTangents()...)

			if len(got) != len(want) {
				t.Fatalf("tangent count: native %d != bridge %d", len(got), len(want))
			}
			for i := range got {
				if math.Abs(got[i]-want[i]) > 1e-9 {
					t.Fatalf("tangent[%d] (halfedge %d, comp %d): native %v != bridge %v",
						i, i/4, i%4, got[i], want[i])
				}
			}
		})
	}
}
