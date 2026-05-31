package manifold

import (
	"runtime"
	"testing"

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
