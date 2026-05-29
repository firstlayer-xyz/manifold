package manifold

import (
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// TestBoolean3Ctor_Sanity exercises the native Boolean3 ctor (Intersect12 +
// Winding03, both forward and reverse) on two overlapping cubes. It's the first
// RUNTIME exercise of the forward=false / expandP=true paths (RayCast only hit
// forward=true / expandP=false). Full differential validation comes at Phase 2
// (Boolean3::Result vs the bridge Boolean); here we assert it runs and produces
// structurally sane output. The Impl views must outlive the Operands (their
// slices alias C++ memory).
func TestBoolean3Ctor_Sanity(t *testing.T) {
	ma := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)  // [-1,1]^3
	mb := Cube(Vec3{X: 2, Y: 2, Z: 2}, false) // [0,2]^3, overlaps in [0,1]^3
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)
	va := getImpl(ma)
	defer va.Delete()
	vb := getImpl(mb)
	defer vb.Delete()

	makeOp := func(v *Impl) boolean.Operand {
		min, max := v.BBox()
		fb, fm := v.GetFaceBoxMorton()
		return boolean.Operand{
			VertPos:    v.Verts(),
			VertNormal: v.VertNormals(),
			FaceNormal: v.FaceNormals(),
			Starts:     v.HalfedgeStarts(),
			Pairs:      v.HalfedgePairs(),
			BBox:       geom.Box{Min: min, Max: max},
			Collider:   collider.New(fb, fm),
		}
	}
	pOp := makeOp(va)
	qOp := makeOp(vb)

	for _, expandP := range []bool{true, false} {
		b3 := boolean.NewBoolean3(pOp, qOp, expandP)
		if !b3.Valid() {
			t.Fatalf("expandP=%v: Boolean3 not valid", expandP)
		}
		_, x12, _ := b3.XV12()
		_, x21, _ := b3.XV21()
		if len(x12) == 0 || len(x21) == 0 {
			t.Errorf("expandP=%v: expected edge/face crossings, got xv12=%d xv21=%d", expandP, len(x12), len(x21))
		}
		if len(b3.W03()) != len(pOp.VertPos) || len(b3.W30()) != len(qOp.VertPos) {
			t.Errorf("expandP=%v: winding lengths w03=%d (want %d) w30=%d (want %d)",
				expandP, len(b3.W03()), len(pOp.VertPos), len(b3.W30()), len(qOp.VertPos))
		}
		// Winding of a point vs a convex cube is 0 or +/-1; allow a small margin.
		for i, w := range b3.W03() {
			if w < -2 || w > 2 {
				t.Errorf("expandP=%v: w03[%d]=%d out of sane range", expandP, i, w)
			}
		}
	}
}

// TestBoolean3Ctor_Windings checks Winding03 against analytic ground truth:
// for A=[-1,1]^3 vs B=[0,2]^3, exactly one A-vert (1,1,1) is strictly inside B
// and exactly one B-vert (0,0,0) is strictly inside A, so w03 and w30 each have
// exactly one nonzero winding, at that interior vert. This directly guards the
// collider point-query / flood-fill that previously returned all-zero windings.
func TestBoolean3Ctor_Windings(t *testing.T) {
	ma := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)  // [-1,1]^3
	mb := Cube(Vec3{X: 2, Y: 2, Z: 2}, false) // [0,2]^3
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)
	va := getImpl(ma)
	defer va.Delete()
	vb := getImpl(mb)
	defer vb.Delete()
	mk := func(v *Impl) boolean.Operand {
		mn, mx := v.BBox()
		fb, fm := v.GetFaceBoxMorton()
		return boolean.Operand{VertPos: v.Verts(), VertNormal: v.VertNormals(), FaceNormal: v.FaceNormals(),
			Starts: v.HalfedgeStarts(), Pairs: v.HalfedgePairs(), PropVert: v.HalfedgeProps(),
			BBox: geom.Box{Min: mn, Max: mx}, Collider: collider.New(fb, fm)}
	}
	p, q := mk(va), mk(vb)

	// strictlyInside reports whether all coords of pt are in the open box (lo,hi).
	strictlyInside := func(pt geom.Vec3, lo, hi float64) bool {
		return pt.X > lo && pt.X < hi && pt.Y > lo && pt.Y < hi && pt.Z > lo && pt.Z < hi
	}
	// checkOne asserts exactly one nonzero winding, at a vert that is strictly
	// inside the other solid's open box.
	checkOne := func(name string, w []int, verts []geom.Vec3, lo, hi float64) {
		nz := []int{}
		for i, x := range w {
			if x != 0 {
				nz = append(nz, i)
			}
		}
		if len(nz) != 1 {
			t.Fatalf("%s: expected exactly 1 nonzero winding, got %d (%v)", name, len(nz), w)
		}
		if !strictlyInside(verts[nz[0]], lo, hi) {
			t.Errorf("%s: nonzero winding at vert %d=%v, not strictly inside (%g,%g)", name, nz[0], verts[nz[0]], lo, hi)
		}
	}
	for _, expandP := range []bool{true, false} {
		b3 := boolean.NewBoolean3(p, q, expandP)
		checkOne("w03", b3.W03(), p.VertPos, 0, 2)  // A-vert inside B=[0,2]
		checkOne("w30", b3.W30(), q.VertPos, -1, 1) // B-vert inside A=[-1,1]
	}
}

// TestBoolean3Result_Sanity exercises the entire native Result pipeline
// (assemble -> face2Tri -> reorderHalfedges) at runtime on two overlapping
// cubes, for all three ops. It asserts the pre-SimplifyTopology mesh is a
// consistently-paired manifold (the C++ DEBUG_ASSERTs IsManifold at this point)
// with one normal/ref per triangle. Full differential validation vs the bridge
// comes once the Impl finalize tail is wired.
func TestBoolean3Result_Sanity(t *testing.T) {
	ma := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)  // [-1,1]^3
	mb := Cube(Vec3{X: 2, Y: 2, Z: 2}, false) // [0,2]^3
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)
	va := getImpl(ma)
	defer va.Delete()
	vb := getImpl(mb)
	defer vb.Delete()

	makeOp := func(v *Impl) boolean.Operand {
		min, max := v.BBox()
		fb, fm := v.GetFaceBoxMorton()
		return boolean.Operand{
			VertPos:    v.Verts(),
			VertNormal: v.VertNormals(),
			FaceNormal: v.FaceNormals(),
			Starts:     v.HalfedgeStarts(),
			Pairs:      v.HalfedgePairs(),
			PropVert:   v.HalfedgeProps(),
			BBox:       geom.Box{Min: min, Max: max},
			Collider:   collider.New(fb, fm),
		}
	}
	pOp := makeOp(va)
	qOp := makeOp(vb)

	cases := []struct {
		name    string
		op      int
		expandP bool
	}{
		{"Add", boolean.OpAdd, true},
		{"Subtract", boolean.OpSubtract, false},
		{"Intersect", boolean.OpIntersect, false},
	}
	for _, tc := range cases {
		b3 := boolean.NewBoolean3(pOp, qOp, tc.expandP)
		rm, ok := b3.Result(tc.op)
		if !ok {
			t.Fatalf("%s: Result returned empty", tc.name)
		}
		n := len(rm.Starts)
		if n == 0 || n%3 != 0 {
			t.Fatalf("%s: bad halfedge count %d", tc.name, n)
		}
		numTri := n / 3
		if len(rm.TriNormal) != numTri || len(rm.TriRef) != numTri {
			t.Errorf("%s: triNormal=%d triRef=%d, want %d each", tc.name, len(rm.TriNormal), len(rm.TriRef), numTri)
		}
		// Consistently-paired manifold: Start(Pair(e))==End(e), Pair(Pair(e))==e.
		end := func(e int) int32 { return rm.Starts[3*(e/3)+(e+1)%3] }
		for e := 0; e < n; e++ {
			p := int(rm.Pairs[e])
			if p < 0 || p >= n {
				t.Fatalf("%s: edge %d pair %d out of range", tc.name, e, p)
			}
			if rm.Starts[p] != end(e) {
				t.Errorf("%s: edge %d Start(Pair)=%d != End=%d", tc.name, e, rm.Starts[p], end(e))
			}
			if int(rm.Pairs[p]) != e {
				t.Errorf("%s: edge %d Pair(Pair)=%d != %d", tc.name, e, rm.Pairs[p], e)
			}
		}
	}
}
