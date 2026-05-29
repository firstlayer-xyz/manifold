package triangulate

import (
	"math"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

func squarePolysIdx() PolygonsIdx {
	return PolygonsIdx{{
		{Pos: geom.Vec2{X: 0, Y: 0}, Idx: 0},
		{Pos: geom.Vec2{X: 1, Y: 0}, Idx: 1},
		{Pos: geom.Vec2{X: 1, Y: 1}, Idx: 2},
		{Pos: geom.Vec2{X: 0, Y: 1}, Idx: 3},
	}}
}

// TestEarClip_InitializeRing: a clean square builds a 4-vert circular list
// with correct links, rightDir, and bounding box, and no ear is clipped.
func TestEarClip_InitializeRing(t *testing.T) {
	ec := newEarClip(squarePolysIdx(), 1e-9)
	if len(ec.polygon) != 4 {
		t.Fatalf("polygon len = %d, want 4", len(ec.polygon))
	}
	for i := range ec.polygon {
		if clipped(&ec.polygon[i]) {
			t.Errorf("vert %d clipped after init of a clean square", i)
		}
	}
	// Loop visits every un-clipped vert exactly once.
	var order []int
	ec.loop(&ec.polygon[0], func(v *vert) { order = append(order, v.meshIdx) })
	if len(order) != 4 || order[0] != 0 {
		t.Fatalf("loop visited %v, want 4 verts starting at 0", order)
	}
	// rightDir of vert0: (0,0)->(1,0) normalized = (1,0).
	if d := ec.polygon[0].rightDir; math.Abs(d.X-1) > 1e-12 || math.Abs(d.Y) > 1e-12 {
		t.Errorf("rightDir[0] = %v, want {1,0}", d)
	}
	if ec.bBox.Min != (geom.Vec2{X: 0, Y: 0}) || ec.bBox.Max != (geom.Vec2{X: 1, Y: 1}) {
		t.Errorf("bBox = %v", ec.bBox)
	}
}

// TestEarClip_EpsilonResolution: epsilon=-1 resolves to bBox.Scale()*kPrecision.
func TestEarClip_EpsilonResolution(t *testing.T) {
	ec := newEarClip(squarePolysIdx(), -1)
	want := ec.bBox.Scale() * kPrecision // unit square: Scale()=1
	if ec.epsilon != want {
		t.Fatalf("epsilon = %v, want %v", ec.epsilon, want)
	}
}

// TestEarClip_ClipEar: clipping a vert with three distinct mesh_idx emits the
// (left, self, right) triangle and unlinks the vert.
func TestEarClip_ClipEar(t *testing.T) {
	ec := newEarClip(squarePolysIdx(), 1e-9)
	v1 := &ec.polygon[1] // idx 1, left idx 0, right idx 2
	ec.clipEar(v1)
	if ec.result.NumTri() != 1 {
		t.Fatalf("NumTri = %d, want 1", ec.result.NumTri())
	}
	if tri := ec.result.Triangles()[0]; tri != [3]int{0, 1, 2} {
		t.Fatalf("triangle = %v, want [0 1 2]", tri)
	}
	if !clipped(v1) {
		t.Errorf("v1 should be clipped after clipEar")
	}
}

// TestEarClip_ClipEar_DegenerateFiltered: a duplicated mesh_idx (as hole-vert
// duplication produces) must NOT emit a triangle.
func TestEarClip_ClipEar_DegenerateFiltered(t *testing.T) {
	ec := newEarClip(squarePolysIdx(), 1e-9)
	v1 := &ec.polygon[1]
	v1.left.meshIdx = v1.meshIdx // force a shared mesh_idx
	ec.clipEar(v1)
	if ec.result.NumTri() != 0 {
		t.Errorf("NumTri = %d, want 0 (degenerate filtered)", ec.result.NumTri())
	}
}

// TestEarClip_ClipShortEdge: a vert whose right edge is shorter than epsilon
// is clipped during construction (ClipIfDegenerate).
func TestEarClip_ClipShortEdge(t *testing.T) {
	polys := PolygonsIdx{{
		{Pos: geom.Vec2{X: 0, Y: 0}, Idx: 0},
		{Pos: geom.Vec2{X: 1, Y: 0}, Idx: 1},
		{Pos: geom.Vec2{X: 1, Y: 1e-9}, Idx: 2}, // vert1's right edge is ~1e-9
		{Pos: geom.Vec2{X: 1, Y: 1}, Idx: 3},
		{Pos: geom.Vec2{X: 0, Y: 1}, Idx: 4},
	}}
	ec := newEarClip(polys, 1e-6)
	if !clipped(&ec.polygon[1]) {
		t.Errorf("vert1 (short right edge) should be clipped during construction")
	}
}
