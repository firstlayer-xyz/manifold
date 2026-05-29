package boolean

import (
	"math"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

func TestIntersect(t *testing.T) {
	// Segment a: (0,0,0)->(2,2,0); segment b: (0,2,10)->(2,0,20). Their xy
	// projections cross at (1,1); z_a there is 0, z_b is 15. Hand-derived:
	// dyL=2, dyR=-2 -> useL=false, lambda=-0.5; x=1, y=1, z=0, w=15.
	got := intersect(
		geom.Vec3{X: 0, Y: 0, Z: 0}, geom.Vec3{X: 2, Y: 2, Z: 0},
		geom.Vec3{X: 0, Y: 2, Z: 10}, geom.Vec3{X: 2, Y: 0, Z: 20})
	want := geom.Vec4{X: 1, Y: 1, Z: 0, W: 15}
	const eps = 1e-15
	if math.Abs(got.X-want.X) > eps || math.Abs(got.Y-want.Y) > eps ||
		math.Abs(got.Z-want.Z) > eps || math.Abs(got.W-want.W) > eps {
		t.Fatalf("intersect = %+v, want %+v", got, want)
	}
}

func TestLoadFaceEdges(t *testing.T) {
	// One triangle, halfedges 0,1,2 with start verts 5,2,8 (so the cyclic edges
	// are 5->2, 2->8, 8->5). Pairs are arbitrary distinct values.
	h := halfedges{
		starts: []int32{5, 2, 8},
		pairs:  []int32{10, 11, 12},
	}
	var edge [3]faceEdge
	loadFaceEdges(h, 0, &edge)

	// edge0: 5->2, start>end -> backward, canonical = pair(0)=10, start=2 end=5.
	if edge[0] != (faceEdge{edge: 10, start: 2, end: 5, isForward: false}) {
		t.Errorf("edge0 = %+v", edge[0])
	}
	// edge1: 2->8, start<end -> forward, edge=1.
	if edge[1] != (faceEdge{edge: 1, start: 2, end: 8, isForward: true}) {
		t.Errorf("edge1 = %+v", edge[1])
	}
	// edge2: 8->5, start>end -> backward, canonical = pair(2)=12, start=5 end=8.
	if edge[2] != (faceEdge{edge: 12, start: 5, end: 8, isForward: false}) {
		t.Errorf("edge2 = %+v", edge[2])
	}
}

func TestNextHalfedge(t *testing.T) {
	// Within a triangle: 0->1->2->0; for tri 1: 3->4->5->3.
	got := [6]int{nextHalfedge(0), nextHalfedge(1), nextHalfedge(2), nextHalfedge(3), nextHalfedge(4), nextHalfedge(5)}
	want := [6]int{1, 2, 0, 4, 5, 3}
	if got != want {
		t.Errorf("nextHalfedge = %v, want %v", got, want)
	}
}
