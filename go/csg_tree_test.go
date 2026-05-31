package manifold

import (
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

// TestCompose_VsBridge differential-tests the native compose (CsgLeafNode::
// Compose) against the C++ cppref. For pairwise-disjoint inputs the bridge
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

// TestBatchBoolean_Native_VsReference differential-tests the native n-ary
// BatchBoolean (batchUnion / batchBoolean heap / the Subtract positive-negative
// split) against the C++ bridge cppref. The oracle is built by pairwise
// reduction: Add/Intersect are commutative+associative so their geometry is
// order-independent, and a-b-c == a-(b∪c). Cases mix overlapping and disjoint
// operands and exceed 4 children to exercise the 4-pairs-per-round batching and
// the disjoint-set partition + Compose path inside batchUnion.
func TestBatchBoolean_Native_VsReference(t *testing.T) {
	// refFold reduces handles with f (bridge pairwise), returning the result
	// handle; intermediate handles are released.
	refFold := func(hs []*handle.Manifold, f func(a, b *handle.Manifold) *handle.Manifold) *handle.Manifold {
		acc := hs[0]
		for i := 1; i < len(hs); i++ {
			next := f(acc, hs[i])
			if i > 1 {
				cppref.DeleteManifold(acc)
			}
			acc = next
		}
		return acc
	}
	handlesOf := func(ms []*Manifold) []*handle.Manifold {
		hs := make([]*handle.Manifold, len(ms))
		for i, m := range ms {
			hs[i] = m.refHandle()
		}
		return hs
	}
	assertGeom := func(t *testing.T, got *Manifold, oracle *handle.Manifold) {
		t.Helper()
		if !floatClose(got.Volume(), cppref.Volume(oracle), 1e-8, 1e-8) {
			t.Errorf("Volume: native=%v ref=%v", got.Volume(), cppref.Volume(oracle))
		}
		if !floatClose(got.SurfaceArea(), cppref.SurfaceArea(oracle), 1e-8, 1e-8) {
			t.Errorf("SurfaceArea: native=%v ref=%v", got.SurfaceArea(), cppref.SurfaceArea(oracle))
		}
		if g, r := got.Genus(), cppref.Genus(oracle); g != r {
			t.Errorf("Genus: native=%d ref=%d", g, r)
		}
	}

	t.Run("union_mixed_5", func(t *testing.T) {
		// Two overlapping clusters + a disjoint one: exercises Compose (disjoint
		// sets) and batchBoolean (overlapping) within batchUnion.
		ms := []*Manifold{
			Cube(Vec3{X: 1, Y: 1, Z: 1}, true),
			Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 0.5}),
			Sphere(0.7, 24).Translate(Vec3{X: 0.3, Y: 0.3}),
			Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 8}), // disjoint
			Sphere(0.6, 20).Translate(Vec3{X: 8.2, Y: 0.2, Z: 0.2}),  // overlaps #4
		}
		for _, m := range ms {
			defer runtime.KeepAlive(m)
		}
		got := BatchBoolean(ms, OpAdd)
		defer runtime.KeepAlive(got)
		oracle := refFold(handlesOf(ms), cppref.Union)
		defer cppref.DeleteManifold(oracle)
		assertGeom(t, got, oracle)
	})

	t.Run("intersect_3", func(t *testing.T) {
		ms := []*Manifold{
			Sphere(1.0, 32),
			Sphere(1.0, 32).Translate(Vec3{X: 0.6}),
			Sphere(1.0, 32).Translate(Vec3{Y: 0.6}),
		}
		for _, m := range ms {
			defer runtime.KeepAlive(m)
		}
		got := BatchBoolean(ms, OpIntersect)
		defer runtime.KeepAlive(got)
		oracle := refFold(handlesOf(ms), cppref.Intersection)
		defer cppref.DeleteManifold(oracle)
		assertGeom(t, got, oracle)
	})

	t.Run("subtract_a_minus_bc", func(t *testing.T) {
		a := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
		b := Sphere(0.8, 24).Translate(Vec3{X: 0.6})
		c := Sphere(0.8, 24).Translate(Vec3{X: -0.6})
		for _, m := range []*Manifold{a, b, c} {
			defer runtime.KeepAlive(m)
		}
		got := BatchBoolean([]*Manifold{a, b, c}, OpSubtract)
		defer runtime.KeepAlive(got)
		// a - b - c == a - (b ∪ c)
		bc := cppref.Union(b.refHandle(), c.refHandle())
		defer cppref.DeleteManifold(bc)
		oracle := cppref.Difference(a.refHandle(), bc)
		defer cppref.DeleteManifold(oracle)
		assertGeom(t, got, oracle)
	})
}
