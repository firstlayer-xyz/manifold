package quickhull

import (
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// TestQuickHull_Tetrahedron: 4 points → tetrahedron with 4
// triangles = 12 halfedges.
func TestQuickHull_Tetrahedron(t *testing.T) {
	pts := []geom.Vec3{
		{X: 1, Y: 1, Z: 1},
		{X: -1, Y: -1, Z: 1},
		{X: -1, Y: 1, Z: -1},
		{X: 1, Y: -1, Z: -1},
	}
	q := NewQuickHull(pts)
	hes, verts := q.BuildMesh(DefaultEpsilon)
	if len(hes) != 12 {
		t.Errorf("tetrahedron halfedges: got %d, want 12", len(hes))
	}
	if len(verts) != 4 {
		t.Errorf("tetrahedron vertices: got %d, want 4", len(verts))
	}
}

// TestQuickHull_Cube: 8 corners → 12-triangle cube hull = 36
// halfedges. Vertex count is 8.
func TestQuickHull_Cube(t *testing.T) {
	pts := []geom.Vec3{
		{X: -1, Y: -1, Z: -1},
		{X: -1, Y: -1, Z: 1},
		{X: -1, Y: 1, Z: -1},
		{X: -1, Y: 1, Z: 1},
		{X: 1, Y: -1, Z: -1},
		{X: 1, Y: -1, Z: 1},
		{X: 1, Y: 1, Z: -1},
		{X: 1, Y: 1, Z: 1},
	}
	q := NewQuickHull(pts)
	hes, verts := q.BuildMesh(DefaultEpsilon)
	if len(hes) != 36 {
		t.Errorf("cube halfedges: got %d, want 36 (12 triangles)", len(hes))
	}
	if len(verts) != 8 {
		t.Errorf("cube vertices: got %d, want 8", len(verts))
	}
}

// TestQuickHull_InteriorPointDropped: cube + one interior point.
// The interior point must not appear in the output.
func TestQuickHull_InteriorPointDropped(t *testing.T) {
	pts := []geom.Vec3{
		{X: -1, Y: -1, Z: -1},
		{X: -1, Y: -1, Z: 1},
		{X: -1, Y: 1, Z: -1},
		{X: -1, Y: 1, Z: 1},
		{X: 1, Y: -1, Z: -1},
		{X: 1, Y: -1, Z: 1},
		{X: 1, Y: 1, Z: -1},
		{X: 1, Y: 1, Z: 1},
		{X: 0, Y: 0, Z: 0}, // interior — must be dropped
	}
	q := NewQuickHull(pts)
	_, verts := q.BuildMesh(DefaultEpsilon)
	if len(verts) != 8 {
		t.Errorf("cube+interior vertices: got %d, want 8 (interior dropped)", len(verts))
	}
}
