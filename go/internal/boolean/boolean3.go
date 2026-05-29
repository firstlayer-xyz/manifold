package boolean

import (
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// intMax is std::numeric_limits<int>::max() (2^31 - 1), the C++ overflow guard
// on the crossing count.
const intMax = int(^uint32(0) >> 1)

// Operand is the input for one Boolean3 mesh: the arrays the kernel reads, the
// bounding box (for the overlap early-out), and the face collider
// (collider.New(GetFaceBoxMorton()), leaf == face index).
type Operand struct {
	VertPos    []geom.Vec3
	VertNormal []geom.Vec3
	FaceNormal []geom.Vec3
	Starts     []int32
	Pairs      []int32
	PropVert   []int32
	BBox       geom.Box
	Collider   *collider.Collider
	Epsilon    float64
	Tolerance  float64
}

func (o Operand) mesh() *mesh {
	return &mesh{
		vertPos:    o.VertPos,
		vertNormal: o.VertNormal,
		faceNormal: o.FaceNormal,
		halfedge:   halfedges{starts: o.Starts, pairs: o.Pairs, propVert: o.PropVert},
	}
}

// Boolean3 is the Go port of the C++ Boolean3 class state (boolean3.h:58): the
// edge<->face crossings of P-vs-Q (xv12) and Q-vs-P (xv21), and the per-vertex
// winding numbers w03 (P verts vs Q) and w30 (Q verts vs P). Result (Phase 2)
// turns these into the output mesh.
type Boolean3 struct {
	p, q       Operand // the C++ const Impl& inP_, inQ_ (Result reads them)
	xv12, xv21 intersections
	w03, w30   []int
	expandP    bool
	valid      bool
}

// NewBoolean3 is the Go port of the Boolean3 ctor (boolean3.cpp:496). expandP
// is op == OpType::Add (Union expands both operands; Difference/Intersection
// contract P, expand Q). It computes the four intersection/winding arrays via
// Intersect12<true/false> and Winding03<true/false>.
func NewBoolean3(p, q Operand, expandP bool) *Boolean3 {
	b := &Boolean3{p: p, q: q, expandP: expandP, valid: true}
	pMesh, qMesh := p.mesh(), q.mesh()

	// No overlap (either empty, or disjoint bboxes) -> all windings 0.
	if len(p.Starts) == 0 || len(q.Starts) == 0 || !p.BBox.DoesOverlap(q.BBox) {
		b.w03 = make([]int, len(p.VertPos))
		b.w30 = make([]int, len(q.VertPos))
		return b
	}

	// Edge/face crossings: forward (edgeP/faceQ) queries Q's collider; reverse
	// (edgeQ/faceP) queries P's collider.
	b.xv12 = intersect12(pMesh, qMesh, q.Collider, expandP, true)
	b.xv21 = intersect12(pMesh, qMesh, p.Collider, expandP, false)

	if len(b.xv12.x12) > intMax || len(b.xv21.x12) > intMax {
		b.valid = false
		return b
	}

	// Winding numbers via flood fill (same forward/collider correspondence).
	b.w03 = winding03(pMesh, qMesh, b.xv12.p1q2, q.Collider, expandP, true)
	b.w30 = winding03(pMesh, qMesh, b.xv21.p1q2, p.Collider, expandP, false)
	return b
}

// Accessors for differential/sanity testing (the C++ members are private).
func (b *Boolean3) XV12() ([][2]int, []int, []geom.Vec3) {
	return b.xv12.p1q2, b.xv12.x12, b.xv12.v12
}
func (b *Boolean3) XV21() ([][2]int, []int, []geom.Vec3) {
	return b.xv21.p1q2, b.xv21.x12, b.xv21.v12
}
func (b *Boolean3) W03() []int  { return b.w03 }
func (b *Boolean3) W30() []int  { return b.w30 }
func (b *Boolean3) Valid() bool { return b.valid }
