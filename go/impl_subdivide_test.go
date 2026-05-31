package manifold

import (
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
)

// TestSubdivideHelpers_Cube validates the subdivision topology helpers on a cube
// (all triangles, no tangent-marked quads): GetNeighbor is always -1, GetHalfedges
// returns the tri's three halfedges + -1, GetIndices returns {tri, i, Next3(i)},
// and CreateTmpEdges yields one forward edge per pair. The quad paths need
// tangent-marked meshes and are covered later via the Subdivide differential.
func TestSubdivideHelpers_Cube(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, false)
	defer runtime.KeepAlive(m)
	view := getImpl(m)
	defer view.Delete()
	mi := view.Copy()
	defer mi.Delete()

	numTri := mi.NumTri()
	if numTri != 12 {
		t.Fatalf("cube NumTri = %d, want 12", numTri)
	}
	for tri := 0; tri < numTri; tri++ {
		if n := mi.getNeighbor(tri); n != -1 {
			t.Errorf("getNeighbor(%d) = %d, want -1 (cube has no quads)", tri, n)
		}
		got := mi.getHalfedges(tri)
		want := [4]int{3 * tri, 3*tri + 1, 3*tri + 2, -1}
		if got != want {
			t.Errorf("getHalfedges(%d) = %v, want %v", tri, got, want)
		}
		for i := 0; i < 3; i++ {
			bi := mi.getIndices(3*tri + i)
			if bi.Tri != tri || bi.Start4 != i || bi.End4 != geom.Next3(i) {
				t.Errorf("getIndices(%d) = %+v, want {%d %d %d}", 3*tri+i, bi, tri, i, geom.Next3(i))
			}
		}
	}

	edges := mesh.CreateTmpEdges(&mi.s.halfedge)
	if len(edges) != mi.HalfedgeCount()/2 {
		t.Errorf("CreateTmpEdges count = %d, want %d", len(edges), mi.HalfedgeCount()/2)
	}
	for _, e := range edges {
		if e.First > e.Second {
			t.Errorf("CreateTmpEdges edge not normalized: %+v", e)
		}
		if !mi.s.halfedge.IsForward(e.HalfedgeIdx) {
			t.Errorf("CreateTmpEdges edge %+v references a non-forward halfedge", e)
		}
	}
}

// TestSubdivide_VsBridge differential-tests the native topological Subdivide (via
// SubdivideN) against the C++ bridge on the same inputs. Subdivision is fully
// deterministic (the Partition triangulation is purely topological), so the
// output vert/tri counts must match exactly and the geometry to fp precision.
func TestSubdivide_VsBridge(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
		n    int
	}{
		{"cube_n2", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, true) }, 2},
		{"cube_n5", func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, true) }, 5},
		{"tetra_n4", func() *Manifold { return fromRefHandle(cppref.Tetrahedron()) }, 4},
		{"sphere_n3", func() *Manifold { return Sphere(1, 8) }, 3},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			defer runtime.KeepAlive(m)

			nv := getImpl(m).Copy()
			defer nv.Delete()
			nv.SubdivideN(tc.n)
			got := nv.ToManifold()
			defer runtime.KeepAlive(got)

			bv := getImpl(m).Copy()
			defer bv.Delete()
			runCppAlgo(bv, func(bm *cppref.MutableImpl) { bm.SubdivideN(tc.n) })
			want := bv.ToManifold()
			defer runtime.KeepAlive(want)

			if g, w := got.NumVert(), want.NumVert(); g != w {
				t.Errorf("NumVert: native=%d bridge=%d", g, w)
			}
			if g, w := got.NumTri(), want.NumTri(); g != w {
				t.Errorf("NumTri: native=%d bridge=%d", g, w)
			}
			if !floatClose(got.Volume(), want.Volume(), 1e-9, 1e-9) {
				t.Errorf("Volume: native=%v bridge=%v", got.Volume(), want.Volume())
			}
			if !floatClose(got.SurfaceArea(), want.SurfaceArea(), 1e-9, 1e-9) {
				t.Errorf("SurfaceArea: native=%v bridge=%v", got.SurfaceArea(), want.SurfaceArea())
			}
		})
	}
}
