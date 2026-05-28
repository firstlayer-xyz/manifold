package manifold

import (
	"runtime"
	"testing"
)

// TestDedupeEdges_NoOpOnPrimitive: primitive shapes have no
// duplicate edges (every pair of halfedges in a fan goes to a
// distinct end vertex), so DedupeEdges must not add any vertices
// or change halfedge count.
func TestDedupeEdges_NoOpOnPrimitive(t *testing.T) {
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
			src := getImpl(tc.m)
			defer src.Delete()
			mi := src.Copy()
			defer mi.Delete()
			vBefore := mi.NumVert()
			hBefore := mi.HalfedgeCount()
			mi.DedupeEdges()
			if mi.NumVert() != vBefore {
				t.Errorf("%s: DedupeEdges grew NumVert (%d → %d) on non-duplicate mesh",
					tc.name, vBefore, mi.NumVert())
			}
			if mi.HalfedgeCount() != hBefore {
				t.Errorf("%s: DedupeEdges grew HalfedgeCount (%d → %d) on non-duplicate mesh",
					tc.name, hBefore, mi.HalfedgeCount())
			}
			if !mi.IsManifold() {
				t.Errorf("%s: DedupeEdges broke IsManifold", tc.name)
			}
		})
	}
}

// TestCleanupTopology_NoOpOnPrimitive: CleanupTopology = SplitPinchedVerts +
// DedupeEdges. Neither does anything on a fresh primitive — so
// CleanupTopology must be a no-op too.
func TestCleanupTopology_NoOpOnPrimitive(t *testing.T) {
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
			src := getImpl(tc.m)
			defer src.Delete()
			mi := src.Copy()
			defer mi.Delete()
			vBefore := mi.NumVert()
			hBefore := mi.HalfedgeCount()
			mi.CleanupTopology()
			if mi.NumVert() != vBefore || mi.HalfedgeCount() != hBefore {
				t.Errorf("%s: CleanupTopology changed counts on a clean mesh "+
					"(verts %d→%d, halfedges %d→%d)",
					tc.name, vBefore, mi.NumVert(), hBefore, mi.HalfedgeCount())
			}
			if !mi.IsManifold() {
				t.Errorf("%s: CleanupTopology broke IsManifold", tc.name)
			}
		})
	}
}

// TestDedupeEdges_ParallelBranch exercises the parallel detection
// branch (nbEdges > 10000) by using a high-resolution sphere.
// Must remain a no-op since spheres have no duplicate edges.
func TestDedupeEdges_ParallelBranch(t *testing.T) {
	m := Sphere(1.0, 192)
	defer runtime.KeepAlive(m)
	src := getImpl(m)
	defer src.Delete()
	if src.HalfedgeCount() <= 10000 {
		t.Skip("sphere too small to exercise parallel branch")
	}
	mi := src.Copy()
	defer mi.Delete()
	hBefore := mi.HalfedgeCount()
	mi.DedupeEdges()
	if mi.HalfedgeCount() != hBefore {
		t.Errorf("parallel DedupeEdges grew halfedges (%d → %d) on clean sphere",
			hBefore, mi.HalfedgeCount())
	}
	if !mi.IsManifold() {
		t.Error("parallel DedupeEdges broke IsManifold")
	}
}

// TestSplitPinchedVerts_ParallelBranch exercises the parallel
// branch (nbEdges > 10000) by using a high-resolution sphere. Must
// still be a no-op since spheres have no pinched verts.
func TestSplitPinchedVerts_ParallelBranch(t *testing.T) {
	m := Sphere(1.0, 192) // ~24576 tris, ~73728 halfedges → parallel branch
	defer runtime.KeepAlive(m)
	src := getImpl(m)
	defer src.Delete()
	if src.HalfedgeCount() <= 10000 {
		t.Skip("sphere too small to exercise parallel branch")
	}
	mi := src.Copy()
	defer mi.Delete()
	before := mi.NumVert()
	mi.SplitPinchedVerts()
	if mi.NumVert() != before {
		t.Errorf("parallel SplitPinchedVerts added %d verts on clean sphere", mi.NumVert()-before)
	}
	if !mi.IsManifold() {
		t.Error("parallel SplitPinchedVerts broke IsManifold")
	}
}

// TestSplitPinchedVerts_NoOpOnPrimitive: primitive shapes have no
// pinched verts (every vertex has exactly one fan of incident
// halfedges), so SplitPinchedVerts must not add any new vertices.
func TestSplitPinchedVerts_NoOpOnPrimitive(t *testing.T) {
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
			src := getImpl(tc.m)
			defer src.Delete()
			mi := src.Copy()
			defer mi.Delete()
			before := mi.NumVert()
			mi.SplitPinchedVerts()
			after := mi.NumVert()
			if before != after {
				t.Errorf("%s: SplitPinchedVerts added %d verts on a non-pinched mesh",
					tc.name, after-before)
			}
			if !mi.IsManifold() {
				t.Errorf("%s: SplitPinchedVerts broke IsManifold", tc.name)
			}
		})
	}
}
