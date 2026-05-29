package boolean

import "testing"

// TestAssembleHalfedges walks a single 3-edge loop and a 4-edge loop into the
// expected halfedge-index polygons, honoring startHalfedgeIdx.
func TestAssembleHalfedges(t *testing.T) {
	// Triangle: 0->1, 1->2, 2->0.
	tri := []Halfedge{
		{StartVert: 0, EndVert: 1},
		{StartVert: 1, EndVert: 2},
		{StartVert: 2, EndVert: 0},
	}
	got := assembleHalfedges(tri, 0, 3, 0)
	if len(got) != 1 || len(got[0]) != 3 || got[0][0] != 0 || got[0][1] != 1 || got[0][2] != 2 {
		t.Fatalf("triangle loop = %v, want [[0 1 2]]", got)
	}
	// Same, offset by startHalfedgeIdx = 10.
	got = assembleHalfedges(tri, 0, 3, 10)
	if len(got) != 1 || got[0][0] != 10 || got[0][1] != 11 || got[0][2] != 12 {
		t.Fatalf("offset loop = %v, want [[10 11 12]]", got)
	}

	// Quad: 0->1, 1->2, 2->3, 3->0.
	quad := []Halfedge{
		{StartVert: 0, EndVert: 1},
		{StartVert: 1, EndVert: 2},
		{StartVert: 2, EndVert: 3},
		{StartVert: 3, EndVert: 0},
	}
	got = assembleHalfedges(quad, 0, 4, 0)
	if len(got) != 1 || len(got[0]) != 4 || got[0][0] != 0 || got[0][3] != 3 {
		t.Fatalf("quad loop = %v, want [[0 1 2 3]]", got)
	}
}

// TestWriteLocalTriangles emits a single triangle: three contour halfedges with
// the face's start/prop verts, no interior pairs, recorded in contour2Tri.
func TestWriteLocalTriangles(t *testing.T) {
	faceHalfedge := []Halfedge{
		{StartVert: 5, EndVert: 6, PropVert: 50},
		{StartVert: 6, EndVert: 7, PropVert: 60},
		{StartVert: 7, EndVert: 5, PropVert: 70},
	}
	out := halfedges{starts: make([]int32, 3), pairs: make([]int32, 3), propVert: make([]int32, 3)}
	contour2Tri := []int{-1, -1, -1}

	writeLocalTriangles(out, contour2Tri, faceHalfedge, 0, [][3]int{{0, 1, 2}}, 1)

	for i, wantStart := range []int{5, 6, 7} {
		if out.Start(i) != wantStart {
			t.Errorf("out.Start(%d) = %d, want %d", i, out.Start(i), wantStart)
		}
		if out.Prop(i) != []int{50, 60, 70}[i] {
			t.Errorf("out.Prop(%d) = %d, want %d", i, out.Prop(i), []int{50, 60, 70}[i])
		}
		if out.Pair(i) != -1 {
			t.Errorf("out.Pair(%d) = %d, want -1 (contour)", i, out.Pair(i))
		}
	}
	// All three edges are contour (no reverse within a single triangle).
	for i, want := range []int{0, 1, 2} {
		if contour2Tri[i] != want {
			t.Errorf("contour2Tri[%d] = %d, want %d", i, contour2Tri[i], want)
		}
	}
}
