package manifold

import (
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// removeDegenResult runs RemoveDegenerates — the native Go port or the C++
// bridge reference — on a copy of m's Impl at the given epsilon/tolerance,
// compacts the result via SortGeometry, and seals it into a Manifold for
// comparison.
func removeDegenResult(t *testing.T, m *Manifold, eps float64, useBridge bool) *Manifold {
	t.Helper()
	view := getImpl(m)
	defer view.Delete()
	impl := view.Copy()
	defer impl.Delete()
	impl.SetEpsilonValue(eps)
	impl.SetToleranceValue(eps)
	if useBridge {
		// The bridge reference reads its storage from the bridge handle, so
		// marshal the native storage in (incl. the eps/tol set above), run the
		// C++ pass, and reload — the same seam production Refine/Subdivide use.
		runCppAlgo(impl, func(bm *cppref.MutableImpl) { bm.RemoveDegenerates(0) })
	} else {
		impl.RemoveDegenerates(0)
	}
	impl.SortGeometry()
	return impl.ToManifold()
}

// TestRemoveDegenerates_VsCpp checks the native Go RemoveDegenerates against
// the C++ cppref. The no-op cases (tiny epsilon) confirm a clean mesh
// passes through identically; the collapse case (epsilon above the edge
// length) exercises CollapseShortEdges / CollapseEdge and friends, and the
// two implementations must agree.
func TestRemoveDegenerates_VsCpp(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
		eps  float64
	}{
		{"cube_noop", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, false) }, 1e-9},
		{"sphere_noop", func() *Manifold { return Sphere(1, 32) }, 1e-9},
		{"sphere_collapse", func() *Manifold { return Sphere(1, 32) }, 0.3},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			defer runtime.KeepAlive(m)
			goM := removeDegenResult(t, m, tc.eps, false)
			defer runtime.KeepAlive(goM)
			refM := removeDegenResult(t, m, tc.eps, true)
			defer runtime.KeepAlive(refM)
			assertSameManifold(t, goM, refM)
		})
	}
}

// assertSameManifold checks that two manifolds are topologically and
// geometrically identical — the differential-equality check for the
// edge_op.cpp ports (Go vs the C++ bridge reference).
func assertSameManifold(t *testing.T, goM, refM *Manifold) {
	t.Helper()
	if g, r := goM.NumVert(), refM.NumVert(); g != r {
		t.Errorf("NumVert: go=%d ref=%d", g, r)
	}
	if g, r := goM.NumTri(), refM.NumTri(); g != r {
		t.Errorf("NumTri: go=%d ref=%d", g, r)
	}
	if g, r := goM.Genus(), refM.Genus(); g != r {
		t.Errorf("Genus: go=%d ref=%d", g, r)
	}
	if !floatClose(goM.Volume(), refM.Volume(), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v", goM.Volume(), refM.Volume())
	}
	if !floatClose(goM.SurfaceArea(), refM.SurfaceArea(), 1e-9, 1e-9) {
		t.Errorf("SurfaceArea: go=%v ref=%v", goM.SurfaceArea(), refM.SurfaceArea())
	}
}

// simplifyTopoResult runs SimplifyTopology — native Go or the C++ bridge
// reference — on a copy of m's Impl, compacts via SortGeometry, and seals
// it into a Manifold.
func simplifyTopoResult(t *testing.T, m *Manifold, eps float64, useBridge bool) *Manifold {
	t.Helper()
	view := getImpl(m)
	defer view.Delete()
	impl := view.Copy()
	defer impl.Delete()
	impl.SetEpsilonValue(eps)
	impl.SetToleranceValue(eps)
	if useBridge {
		runCppAlgo(impl, func(bm *cppref.MutableImpl) { bm.SimplifyTopology() })
	} else {
		impl.SimplifyTopology(0)
	}
	impl.SortGeometry()
	return impl.ToManifold()
}

// TestSimplifyTopology_VsCpp checks native Go SimplifyTopology (which adds
// a CollapseColinearEdges pass to RemoveDegenerates) against the C++
// reference, over no-op and aggressive-collapse cases.
func TestSimplifyTopology_VsCpp(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
		eps  float64
	}{
		{"cube_noop", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, false) }, 1e-9},
		{"sphere_noop", func() *Manifold { return Sphere(1, 32) }, 1e-9},
		{"sphere_collapse", func() *Manifold { return Sphere(1, 32) }, 0.3},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			defer runtime.KeepAlive(m)
			goM := simplifyTopoResult(t, m, tc.eps, false)
			defer runtime.KeepAlive(goM)
			refM := simplifyTopoResult(t, m, tc.eps, true)
			defer runtime.KeepAlive(refM)
			assertSameManifold(t, goM, refM)
		})
	}
}

// TestFlagStore_Seq exercises the serial branch (n <= 1e5): flagged
// indices must be processed in ascending order, exactly once each.
func TestFlagStore_Seq(t *testing.T) {
	const n = 1000
	var fs flagStore
	var got []int
	fs.run(n, func(i int) bool { return i%3 == 0 }, func(i int) { got = append(got, i) })

	var want []int
	for i := 0; i < n; i++ {
		if i%3 == 0 {
			want = append(want, i)
		}
	}
	if len(got) != len(want) {
		t.Fatalf("seq: got %d indices, want %d", len(got), len(want))
	}
	for k := range want {
		if got[k] != want[k] {
			t.Fatalf("seq: index %d = %d, want %d (order must be ascending)", k, got[k], want[k])
		}
	}
}

// TestFlagStore_Par exercises the parallel branch (n > 1e5). The predicate
// runs concurrently, but f must still see ascending order, once each.
func TestFlagStore_Par(t *testing.T) {
	const n = 250000 // > 1e5 -> runPar
	var fs flagStore
	var got []int
	// f runs serially per the FlagStore contract, so no lock is needed.
	fs.run(n, func(i int) bool { return i%7 == 0 }, func(i int) { got = append(got, i) })

	var want []int
	for i := 0; i < n; i++ {
		if i%7 == 0 {
			want = append(want, i)
		}
	}
	if len(got) != len(want) {
		t.Fatalf("par: got %d indices, want %d", len(got), len(want))
	}
	for k := range want {
		if got[k] != want[k] {
			t.Fatalf("par: index %d = %d, want %d (must be ascending after merge+sort)", k, got[k], want[k])
		}
	}
}

// TestFlagStore_Reuse verifies the store resets between runs.
func TestFlagStore_Reuse(t *testing.T) {
	var fs flagStore
	var first []int
	fs.run(10, func(i int) bool { return i < 3 }, func(i int) { first = append(first, i) })
	var second []int
	fs.run(10, func(i int) bool { return i >= 8 }, func(i int) { second = append(second, i) })

	if len(first) != 3 || first[0] != 0 || first[2] != 2 {
		t.Fatalf("first run = %v, want [0 1 2]", first)
	}
	if len(second) != 2 || second[0] != 8 || second[1] != 9 {
		t.Fatalf("second run = %v, want [8 9]", second)
	}
}
