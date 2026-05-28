package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// TestRemoveUnreferencedVerts_MarksTail builds a cube, copies its
// Impl to a mutable copy, appends two extra verts at the end (which
// no halfedge references), then runs RemoveUnreferencedVerts and
// asserts:
//   - the original verts (referenced by halfedges) are untouched;
//   - the appended verts are now NaN in all three coordinates.
//
// This mirrors the C++ contract: unreferenced verts get NaN'd so
// downstream SortVerts trims them.
func TestRemoveUnreferencedVerts_MarksTail(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, false)
	defer runtime.KeepAlive(m)
	roImpl := getImpl(m)
	defer roImpl.Delete()

	mi := roImpl.Copy()
	defer mi.Delete()

	origNumVert := mi.NumVert()
	origVerts := append([]geom.Vec3(nil), mi.Verts()...)

	// Append two extra verts that no halfedge references.
	mi.h.ResizeVerts(origNumVert + 2)
	verts := mi.Verts()
	verts[origNumVert] = geom.Vec3{X: 99, Y: 99, Z: 99}
	verts[origNumVert+1] = geom.Vec3{X: -7, Y: -7, Z: -7}

	mi.RemoveUnreferencedVerts()

	verts = mi.Verts()
	if len(verts) != origNumVert+2 {
		t.Fatalf("vert count changed: got %d want %d", len(verts), origNumVert+2)
	}

	// Original verts unchanged (each is referenced by some halfedge).
	for i := 0; i < origNumVert; i++ {
		if verts[i] != origVerts[i] {
			t.Errorf("verts[%d]: got %v want %v (referenced vert should be untouched)",
				i, verts[i], origVerts[i])
		}
	}
	// Appended verts NaN'd.
	for i := origNumVert; i < origNumVert+2; i++ {
		if !math.IsNaN(verts[i].X) || !math.IsNaN(verts[i].Y) || !math.IsNaN(verts[i].Z) {
			t.Errorf("verts[%d]: got %v want all-NaN", i, verts[i])
		}
	}
}

// TestRemoveUnreferencedVerts_Empty is the empty-mesh corner case:
// a freshly-constructed Impl with no verts must not crash.
func TestRemoveUnreferencedVerts_Empty(t *testing.T) {
	mi := newImpl()
	defer mi.Delete()
	mi.RemoveUnreferencedVerts() // must not panic
}
