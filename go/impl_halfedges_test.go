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

// TestSetNormalsAndCoplanar_Tetrahedron_VsCpp builds a tetrahedron via
// both the Go-drilled newImplFromShape (which calls Go ports for
// CreateHalfedges, CalculateBBox, SetEpsilon, InitializeOriginal,
// SetNormalsAndCoplanar, CalculateVertNormals) and the C++
// reference, then compares face normals and vertex normals.
//
// Face normals are unique up to triangle ordering — neither path
// guarantees the same tri index for the same triangle (the C++ side
// may reorder via SortGeometry, which both paths still call). To
// compare, we collect (sortKey, normal) pairs and verify the sets
// match within epsilon.
func TestSetNormalsAndCoplanar_Tetrahedron_VsCpp(t *testing.T) {
	mGo := Tetrahedron()
	defer runtime.KeepAlive(mGo)
	goImpl := bridge.GetImpl(mGo.h)
	defer goImpl.Delete()

	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	refImpl := bridge.GetImpl(hRef)
	defer refImpl.Delete()

	// Face normals: number must match.
	goFN := goImpl.FaceNormals()
	refFN := refImpl.FaceNormals()
	if len(goFN) != len(refFN) {
		t.Fatalf("FaceNormals length: go=%d ref=%d", len(goFN), len(refFN))
	}
	// For a tetrahedron, the 4 face normals are unique unit vectors.
	// Check the set of normals (any order) matches.
	if !sameVec3Set(goFN, refFN, 1e-12) {
		t.Errorf("face normal sets differ:\n  go:  %v\n  ref: %v", goFN, refFN)
	}

	// Vertex normals likewise — for a tetrahedron each vertex has a
	// unique normal direction.
	goVN := goImpl.VertNormals()
	refVN := refImpl.VertNormals()
	if len(goVN) != len(refVN) {
		t.Fatalf("VertNormals length: go=%d ref=%d", len(goVN), len(refVN))
	}
	if !sameVec3Set(goVN, refVN, 1e-12) {
		t.Errorf("vert normal sets differ:\n  go:  %v\n  ref: %v", goVN, refVN)
	}
}

func sameVec3Set(a, b []Vec3, eps float64) bool {
	if len(a) != len(b) {
		return false
	}
	used := make([]bool, len(b))
	for _, va := range a {
		matched := false
		for j, vb := range b {
			if used[j] {
				continue
			}
			if vec3Close(va, vb, eps) {
				used[j] = true
				matched = true
				break
			}
		}
		if !matched {
			return false
		}
	}
	return true
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

// TestIs2Manifold_Primitives verifies that primitive shape
// constructors yield 2-manifold meshes (every edge incident to
// exactly two triangles). Stronger than IsManifold which only
// checks halfedge reciprocity.
func TestIs2Manifold_Primitives(t *testing.T) {
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
			if !impl.Is2Manifold() {
				t.Errorf("%s should be Is2Manifold but is not", tc.name)
			}
		})
	}
}

// TestIsManifold_Primitives verifies that all primitive shape
// constructors produce IsManifold-passing meshes. Mirrors the C++
// invariant that newly-constructed primitives are valid manifolds.
func TestIsManifold_Primitives(t *testing.T) {
	cases := []struct {
		name string
		m    *Manifold
	}{
		{"Tetrahedron", Tetrahedron()},
		{"Cube", Cube(Vec3{X: 1, Y: 1, Z: 1}, false)},
		{"Cube centered", Cube(Vec3{X: 2, Y: 3, Z: 4}, true)},
		{"Sphere", Sphere(1.0, 24)},
		{"Cylinder", Cylinder(2.0, 0.5, 0.5, 16, false)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			defer runtime.KeepAlive(tc.m)
			impl := getImpl(tc.m)
			defer impl.Delete()
			if !impl.IsManifold() {
				t.Errorf("%s should be IsManifold but is not", tc.name)
			}
		})
	}
}
