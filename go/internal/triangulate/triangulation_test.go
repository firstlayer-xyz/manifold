package triangulate

import "testing"

// TestHalfedgeTriangulation_SingleTriangle builds one contour + one triangle
// and checks the halfedge mesh: contourEnd, triangle readback, and that every
// halfedge is paired reciprocally with matching reversed endpoints.
func TestHalfedgeTriangulation_SingleTriangle(t *testing.T) {
	tri := newHalfedgeTriangulation()
	polys := PolygonsIdx{{{Idx: 0}, {Idx: 1}, {Idx: 2}}}
	tri.AddContours(polys)
	if tri.ContourEnd != 3 {
		t.Fatalf("ContourEnd = %d, want 3", tri.ContourEnd)
	}
	tri.AddTriangle(0, 1, 2)
	if tri.NumTri() != 1 {
		t.Fatalf("NumTri = %d, want 1", tri.NumTri())
	}
	got := tri.Triangles()
	if len(got) != 1 || got[0] != [3]int{0, 1, 2} {
		t.Fatalf("Triangles = %v, want [[0 1 2]]", got)
	}
	for i, h := range tri.Halfedges {
		if h.PairedHalfedge < 0 {
			t.Fatalf("halfedge %d unpaired", i)
		}
		p := tri.Halfedges[h.PairedHalfedge]
		if p.PairedHalfedge != i {
			t.Errorf("halfedge %d pair not reciprocal", i)
		}
		if h.StartVert != p.EndVert || h.EndVert != p.StartVert {
			t.Errorf("halfedge %d pair endpoints mismatch: %+v vs %+v", i, h, p)
		}
	}
	tri.Finalize()
	if tri.edge2halfedge != nil {
		t.Errorf("Finalize should clear edge2halfedge")
	}
}

func TestEdgeKey(t *testing.T) {
	if got, want := edgeKey(2, 5), uint64(2)<<32|5; got != want {
		t.Fatalf("edgeKey(2,5) = %#x, want %#x", got, want)
	}
	// Negative indices round-trip through uint32 (mirrors the C++ cast).
	if got, want := edgeKey(-1, 0), uint64(0xFFFFFFFF)<<32; got != want {
		t.Fatalf("edgeKey(-1,0) = %#x, want %#x", got, want)
	}
}
