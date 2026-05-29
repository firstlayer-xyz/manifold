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
