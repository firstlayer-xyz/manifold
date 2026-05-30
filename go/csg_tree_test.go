package manifold

import (
	"runtime"
	"testing"
)

// TestCompose_VsBridge differential-tests the native compose (CsgLeafNode::
// Compose) against the C++ bridge. For pairwise-disjoint inputs the bridge
// BatchBoolean(OpAdd) partitions into a single disjoint set and routes through
// Compose, so it is the exact oracle for native compose.
func TestCompose_VsBridge(t *testing.T) {
	cases := []struct {
		name  string
		build func() []*Manifold
	}{
		{"two_cubes", func() []*Manifold {
			return []*Manifold{
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false),
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false).Translate(Vec3{X: 5}),
			}
		}},
		{"three_cubes", func() []*Manifold {
			return []*Manifold{
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false),
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false).Translate(Vec3{X: 5}),
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false).Translate(Vec3{Y: 5}),
			}
		}},
		{"cube_and_sphere", func() []*Manifold {
			return []*Manifold{
				Cube(Vec3{X: 1, Y: 1, Z: 1}, false),
				Sphere(0.8, 24).Translate(Vec3{X: 6}),
			}
		}},
		{"normal_carrying", func() []*Manifold {
			return []*Manifold{
				Sphere(1, 16).CalculateNormals(0, 0),
				Sphere(1, 16).CalculateNormals(0, 0).Translate(Vec3{X: 4}),
			}
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			ms := tc.build()
			for _, m := range ms {
				defer runtime.KeepAlive(m)
			}

			// Native compose over the leaf Impls.
			impls := make([]*Impl, len(ms))
			for i, m := range ms {
				impls[i] = getImpl(m)
				defer impls[i].Delete()
			}
			got := compose(impls).ToManifold()
			defer runtime.KeepAlive(got)

			// Bridge oracle: BatchBoolean(Add) over the disjoint inputs.
			want := BatchBoolean(ms, OpAdd)
			defer runtime.KeepAlive(want)

			assertSameManifold(t, got, want)
			if g, w := got.NumProp(), want.NumProp(); g != w {
				t.Errorf("NumProp: native=%d bridge=%d", g, w)
			}
		})
	}
}
