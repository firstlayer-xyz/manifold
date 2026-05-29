package boolean

import (
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

func TestDuplicateVerts(t *testing.T) {
	// inclusion {1, 2, 0}: vert0 once, vert1 twice, vert2 dropped.
	// vertR {0, 1, 3} = exclusive scan of |inclusion|.
	src := []geom.Vec3{{X: 1}, {X: 2}, {X: 3}}
	inclusion := []int{1, 2, 0}
	vertR := []int{0, 1, 3}
	out := make([]geom.Vec3, 3)
	for v := range src {
		duplicateVerts(out, inclusion, vertR, src, v)
	}
	want := []geom.Vec3{{X: 1}, {X: 2}, {X: 2}}
	for i := range want {
		if out[i] != want[i] {
			t.Fatalf("out = %v, want %v", out, want)
		}
	}
}

func TestCountVerts(t *testing.T) {
	// One triangle, start verts 0,1,2; inclusion -1,2,1 -> |sum| = 1+2+1 = 4.
	h := halfedges{starts: []int32{0, 1, 2}, pairs: []int32{3, 4, 5}}
	inclusion := []int{-1, 2, 1}
	count := []int{0}
	countVerts(h, count, inclusion, 0)
	if count[0] != 4 {
		t.Fatalf("count = %d, want 4", count[0])
	}
}

func TestCountNewVerts(t *testing.T) {
	// edgeP=0 (tri 0), Pair(0)=3 (tri 1), faceQ=2; inclusion |i12[0]|=2.
	h := halfedges{starts: []int32{0, 1, 2, 3, 4, 5}, pairs: []int32{3, 1, 2, 0, 4, 5}}
	countP := make([]int, 4)
	countQ := make([]int, 4)
	pq := [][2]int{{0, 2}} // edgeP=0, faceQ=2
	i12 := []int{2}
	countNewVerts(countP, countQ, i12, pq, h, 0, false)
	if countQ[2] != 2 {
		t.Errorf("countQ[2] = %d, want 2", countQ[2])
	}
	if countP[0] != 2 { // edgeP/3 = 0
		t.Errorf("countP[0] = %d, want 2", countP[0])
	}
	if countP[1] != 2 { // Pair(0)=3 -> 3/3 = 1
		t.Errorf("countP[1] = %d, want 2", countP[1])
	}
}

// TestSizeOutput covers SizeOutput on a trivial 2-triangle case (one P tri, one
// Q tri, all retained verts included once, no new crossings): both faces kept,
// 3 sides each.
func TestSizeOutput(t *testing.T) {
	tri := func() *mesh {
		return &mesh{
			faceNormal: []geom.Vec3{{Z: 1}},
			halfedge:   halfedges{starts: []int32{0, 1, 2}, pairs: []int32{0, 1, 2}},
		}
	}
	inP, inQ := tri(), tri()
	inP.faceNormal = []geom.Vec3{{X: 1}}
	inQ.faceNormal = []geom.Vec3{{Y: 1}}
	var outR outImpl
	i03 := []int{1, 1, 1}
	i30 := []int{1, 1, 1}
	faceEdge, facePQ2R := sizeOutput(&outR, inP, inQ, i03, i30, nil, nil, nil, nil, true)

	if len(outR.faceNormal) != 2 {
		t.Fatalf("faceNormal len = %d, want 2", len(outR.faceNormal))
	}
	if outR.faceNormal[0] != (geom.Vec3{X: 1}) {
		t.Errorf("faceNormal[0] = %v, want P normal {1,0,0}", outR.faceNormal[0])
	}
	if outR.faceNormal[1] != (geom.Vec3{Y: -1}) { // Q negated (invertQ)
		t.Errorf("faceNormal[1] = %v, want negated Q normal {0,-1,0}", outR.faceNormal[1])
	}
	// old face i -> new face index = # kept faces before i. Both kept:
	// inclusive_scan gives [0,1,2], resized to numTriP+numTriQ=2 -> [0,1].
	if len(facePQ2R) != 2 || facePQ2R[0] != 0 || facePQ2R[1] != 1 {
		t.Errorf("facePQ2R = %v, want [0 1]", facePQ2R)
	}
	// Each kept face has 3 sides: faceEdge = [0, 3, 6].
	if len(faceEdge) != 3 || faceEdge[0] != 0 || faceEdge[1] != 3 || faceEdge[2] != 6 {
		t.Errorf("faceEdge = %v, want [0 3 6]", faceEdge)
	}
}

// TestReorderHalfedges builds a valid tetrahedron with one face stored
// off-canonical (not starting at its smallest vert) and asserts that after
// reorderHalfedges every face starts at its smallest vert and the pairing is
// consistent (Start(Pair(e)) == End(e), Pair(Pair(e)) == e).
func TestReorderHalfedges(t *testing.T) {
	// Canonical tetra faces (0,1,2)(0,3,1)(0,2,3)(1,3,2), but face 0 rotated to
	// start at vert 1, with the pairs into face 0 fixed up accordingly.
	starts := []int32{1, 2, 0, 0, 3, 1, 0, 2, 3, 1, 3, 2}
	pairs := []int32{11, 6, 5, 8, 9, 2, 1, 10, 3, 4, 7, 0}
	props := append([]int32(nil), starts...)
	h := halfedges{starts: starts, pairs: pairs, propVert: props}

	reorderHalfedges(h)

	numTri := len(h.starts) / 3
	for tri := 0; tri < numTri; tri++ {
		s0, s1, s2 := h.Start(tri*3), h.Start(tri*3+1), h.Start(tri*3+2)
		if s0 > s1 || s0 > s2 {
			t.Errorf("face %d starts %d,%d,%d not smallest-first", tri, s0, s1, s2)
		}
	}
	for e := 0; e < len(h.starts); e++ {
		if h.Start(h.Pair(e)) != h.End(e) {
			t.Errorf("edge %d: Start(Pair)=%d != End=%d", e, h.Start(h.Pair(e)), h.End(e))
		}
		if h.Pair(h.Pair(e)) != e {
			t.Errorf("edge %d: Pair(Pair)=%d != %d", e, h.Pair(h.Pair(e)), e)
		}
	}
}

// TestPairUp covers the partition + per-half stable-sort + pairing: starts and
// ends interleaved with out-of-order pos, so the sort must reorder each half
// before pairing edgePos[i] with edgePos[i+nEdges].
func TestPairUp(t *testing.T) {
	// starts: vert2@pos1, vert3@pos4 ; ends: vert1@pos3, vert4@pos2.
	// After per-half sort by pos: starts [vert2@1, vert3@4], ends [vert4@2, vert1@3].
	// Pairs: (2,4), (3,1).
	positions := []edgePos{
		{vert: 1, pos: 3, isStart: false},
		{vert: 2, pos: 1, isStart: true},
		{vert: 3, pos: 4, isStart: true},
		{vert: 4, pos: 2, isStart: false},
	}
	var got []Halfedge
	pairUp(positions, func(h Halfedge) { got = append(got, h) })

	want := []Halfedge{
		{StartVert: 2, EndVert: 4, PairedHalfedge: -1},
		{StartVert: 3, EndVert: 1, PairedHalfedge: -1},
	}
	if len(got) != len(want) {
		t.Fatalf("got %d halfedges, want %d: %+v", len(got), len(want), got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("halfedge[%d] = %+v, want %+v", i, got[i], want[i])
		}
	}
}

// TestAddNewEdgeVerts covers one forward crossing: vert lands on P's edge and
// the two new edges (left/right faces of edgeP x faceQ), with isStart per the
// XOR rule and the right keys.
func TestAddNewEdgeVerts(t *testing.T) {
	// edgeP=0 (tri0), Pair(0)=4 (tri1); faceQ=7. inclusion=1 -> direction=false.
	h := halfedges{starts: []int32{0, 1, 2, 3, 4, 5}, pairs: []int32{4, 1, 2, 3, 0, 5}}
	edgesP := map[int][]edgePos{}
	edgesNew := map[[2]int][]edgePos{}
	p1q2 := [][2]int{{0, 7}}
	i12 := []int{1}
	v12R := []int{42}
	addNewEdgeVerts(edgesP, edgesNew, p1q2, i12, v12R, h, true, 0)

	// edgesP[0]: one entry, vert 42, collisionID 0, isStart=direction=false.
	if got := edgesP[0]; len(got) != 1 || got[0].vert != 42 || got[0].collisionID != 0 || got[0].isStart {
		t.Errorf("edgesP[0] = %+v", got)
	}
	// keyLeft = {edgeP/3, faceQ} = {0, 7}; isStart = direction!=forward = false!=true = true.
	if got := edgesNew[[2]int{0, 7}]; len(got) != 1 || got[0].vert != 42 || !got[0].isStart {
		t.Errorf("edgesNew[{0,7}] = %+v", got)
	}
	// keyRight = {Pair(0)/3, faceQ} = {4/3=1, 7} = {1, 7}; isStart = direction!=!forward = false!=false = false.
	if got := edgesNew[[2]int{1, 7}]; len(got) != 1 || got[0].vert != 42 || got[0].isStart {
		t.Errorf("edgesNew[{1,7}] = %+v", got)
	}
}
