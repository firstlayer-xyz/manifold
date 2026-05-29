package manifold

import (
	"math"
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
