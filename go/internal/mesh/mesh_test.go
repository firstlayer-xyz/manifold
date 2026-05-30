package mesh

import (
	"math"
	"testing"
)

func TestNextHalfedge(t *testing.T) {
	for i, want := range []int{1, 2, 0, 4, 5, 3} {
		if got := NextHalfedge(i); got != want {
			t.Errorf("NextHalfedge(%d) = %d, want %d", i, got, want)
		}
	}
}

func TestHalfedgesEndDerived(t *testing.T) {
	// One triangle: 0->1, 1->2, 2->0.
	h := NewHalfedges([]int32{0, 1, 2}, []int32{3, 4, 5}, []int32{10, 11, 12})
	if h.Size() != 3 || h.Empty() {
		t.Fatalf("Size=%d Empty=%v", h.Size(), h.Empty())
	}
	// End(idx) == Start(NextHalfedge(idx)).
	for idx := 0; idx < 3; idx++ {
		if h.End(idx) != h.Start(NextHalfedge(idx)) {
			t.Errorf("End(%d)=%d != Start(next)=%d", idx, h.End(idx), h.Start(NextHalfedge(idx)))
		}
	}
	if h.End(0) != 1 || h.End(2) != 0 {
		t.Errorf("End(0)=%d End(2)=%d, want 1,0", h.End(0), h.End(2))
	}
	if g := h.Get(0); g != (Halfedge{StartVert: 0, EndVert: 1, PairedHalfedge: 3, PropVert: 10}) {
		t.Errorf("Get(0)=%+v", g)
	}
	// Set leaves End derived from the next start.
	h.Set(0, 7, 8, 9)
	if h.Start(0) != 7 || h.Pair(0) != 8 || h.Prop(0) != 9 {
		t.Errorf("after Set: %+v", h.Get(0))
	}
}

func TestHalfedgesResize(t *testing.T) {
	var h Halfedges
	h.Resize(2) // -1 fill
	if h.Start(0) != -1 || h.Pair(1) != -1 {
		t.Errorf("Resize fill: %v", h.Get(0))
	}
	h.PushBack(5, 6, 7)
	if h.Size() != 3 || h.Start(2) != 5 {
		t.Errorf("PushBack: size=%d start[2]=%d", h.Size(), h.Start(2))
	}
	h.ResizeNoFill(1)
	if h.Size() != 1 {
		t.Errorf("ResizeNoFill shrink: size=%d", h.Size())
	}
}

func TestTriRefSameFace(t *testing.T) {
	a := TriRef{MeshID: 1, OriginalID: 2, FaceID: 3, CoplanarID: 4}
	if !a.SameFace(TriRef{MeshID: 1, OriginalID: 99, FaceID: 3, CoplanarID: 4}) {
		t.Error("SameFace should ignore OriginalID")
	}
	if a.SameFace(TriRef{MeshID: 2, OriginalID: 2, FaceID: 3, CoplanarID: 4}) {
		t.Error("SameFace must compare MeshID")
	}
	if a.SameFace(TriRef{MeshID: 1, OriginalID: 2, FaceID: 9, CoplanarID: 4}) {
		t.Error("SameFace must compare FaceID")
	}
}

func TestRelationNormalTransform(t *testing.T) {
	r := NewRelation() // identity, backSide=false
	id := r.GetNormalTransform()
	for c := 0; c < 3; c++ {
		for row := 0; row < 3; row++ {
			want := 0.0
			if c == row {
				want = 1.0
			}
			if math.Abs(id[c][row]-want) > 1e-12 {
				t.Errorf("identity normalTransform[%d][%d]=%g want %g", c, row, id[c][row], want)
			}
		}
	}
	r.BackSide = true
	neg := r.GetNormalTransform()
	if math.Abs(neg[0][0]-(-1.0)) > 1e-12 {
		t.Errorf("backSide normalTransform[0][0]=%g want -1", neg[0][0])
	}
}

func TestMeshRelationDDefault(t *testing.T) {
	mr := NewMeshRelationD()
	if mr.OriginalID != -1 || mr.MeshIDTransform == nil || mr.MeshIDTransform.Len() != 0 {
		t.Errorf("NewMeshRelationD = %+v", mr)
	}
	mr.MeshIDTransform.Set(5, Relation{OriginalID: 5})
	if v, ok := mr.MeshIDTransform.Get(5); !ok || v.OriginalID != 5 {
		t.Errorf("MeshIDTransform.Get(5) = %+v ok=%v", v, ok)
	}
}
