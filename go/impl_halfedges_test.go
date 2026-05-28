package manifold

import (
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/reference"
)

// TestCreateHalfedges_Tetrahedron_VsCpp builds a tetrahedron two ways
// — through the Go-drilled newImplFromShape and through the C++
// constructor (via the reference oracle) — then asserts the resulting
// halfedge layouts agree.
//
// "Agree" here means: identical halfedge_.start_, halfedge_.paired_,
// and matching tri counts. The two paths may differ in *which* index
// pairs a given halfedge to its twin, since the C++ algorithm and
// the Go sort can return different orderings within the same
// equivalence class. So we compare via a canonical structural test:
// every halfedge's paired edge must satisfy the swap-relation
// (start[a] == start[paired[paired[a]]] for a closed mesh).
func TestCreateHalfedges_Tetrahedron_VsCpp(t *testing.T) {
	// Go path: build via newImplFromShape (which calls our drilled
	// createHalfedges internally).
	mGo := Tetrahedron()
	defer runtime.KeepAlive(mGo)
	goImpl := bridge.GetImpl(mGo.h)
	defer goImpl.Delete()
	goStarts := append([]int32(nil), goImpl.HalfedgeStarts()...)
	goPairs := append([]int32(nil), goImpl.HalfedgePairs()...)

	// Reference path: C++ tetrahedron.
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	refImpl := bridge.GetImpl(hRef)
	defer refImpl.Delete()
	refStarts := append([]int32(nil), refImpl.HalfedgeStarts()...)
	refPairs := append([]int32(nil), refImpl.HalfedgePairs()...)

	if len(goStarts) != len(refStarts) {
		t.Fatalf("halfedge count: go=%d ref=%d", len(goStarts), len(refStarts))
	}

	// Structural invariant: for every halfedge i, pairs[pairs[i]] == i,
	// and starts[pairs[i]] == endVert at i.
	assertHalfedgeInvariant(t, "go", goStarts, goPairs)
	assertHalfedgeInvariant(t, "ref", refStarts, refPairs)

	// Edge set equality: the unordered set of (min, max) vertex pairs
	// for forward halfedges must be identical between the two impls.
	if got, want := edgeSet(goStarts, goPairs), edgeSet(refStarts, refPairs); !sameEdgeSet(got, want) {
		t.Errorf("edge set differs:\n  go:  %v\n  ref: %v", got, want)
	}
}

// TestCreateHalfedges_Cube_VsCpp is the cube analog.
func TestCreateHalfedges_Cube_VsCpp(t *testing.T) {
	mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, false)
	defer runtime.KeepAlive(mGo)
	goImpl := bridge.GetImpl(mGo.h)
	defer goImpl.Delete()
	goStarts := append([]int32(nil), goImpl.HalfedgeStarts()...)
	goPairs := append([]int32(nil), goImpl.HalfedgePairs()...)

	hRef := reference.Cube(1, 1, 1, false)
	defer reference.DeleteManifold(hRef)
	refImpl := bridge.GetImpl(hRef)
	defer refImpl.Delete()
	refStarts := append([]int32(nil), refImpl.HalfedgeStarts()...)
	refPairs := append([]int32(nil), refImpl.HalfedgePairs()...)

	if len(goStarts) != len(refStarts) {
		t.Fatalf("halfedge count: go=%d ref=%d", len(goStarts), len(refStarts))
	}
	assertHalfedgeInvariant(t, "go", goStarts, goPairs)
	assertHalfedgeInvariant(t, "ref", refStarts, refPairs)
	if got, want := edgeSet(goStarts, goPairs), edgeSet(refStarts, refPairs); !sameEdgeSet(got, want) {
		t.Errorf("edge set differs:\n  go:  %v\n  ref: %v", got, want)
	}
}

func assertHalfedgeInvariant(t *testing.T, name string, starts, pairs []int32) {
	t.Helper()
	for i := 0; i < len(pairs); i++ {
		p := pairs[i]
		if p < 0 {
			continue
		}
		if int(pairs[p]) != i {
			t.Errorf("%s: pairs[pairs[%d]] = %d, want %d", name, i, pairs[p], i)
		}
		// Triangle structure: halfedge end == start of the next halfedge
		// within the same tri (next = i+1 unless i%3==2 then i-2).
		var next int
		if i%3 == 2 {
			next = i - 2
		} else {
			next = i + 1
		}
		// Paired halfedge's start == this halfedge's end.
		if starts[p] != starts[next] {
			t.Errorf("%s: halfedge %d: starts[pair=%d]=%d but starts[next=%d]=%d",
				name, i, p, starts[p], next, starts[next])
		}
	}
}

func edgeSet(starts, pairs []int32) map[[2]int32]int {
	out := map[[2]int32]int{}
	for i := 0; i < len(pairs); i++ {
		var next int
		if i%3 == 2 {
			next = i - 2
		} else {
			next = i + 1
		}
		s, e := starts[i], starts[next]
		if s > e {
			s, e = e, s
		}
		out[[2]int32{s, e}]++
	}
	return out
}

func sameEdgeSet(a, b map[[2]int32]int) bool {
	if len(a) != len(b) {
		return false
	}
	for k, v := range a {
		if b[k] != v {
			return false
		}
	}
	return true
}
