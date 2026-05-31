package vector

import "testing"

func TestPool_Recycles(t *testing.T) {
	var p Pool[int]

	// Empty pool: Get returns nil; the caller grows it.
	s := p.Get()
	if s != nil {
		t.Fatalf("Get on empty pool should return nil, got len=%d cap=%d", len(s), cap(s))
	}
	for i := 0; i < 8; i++ {
		s = append(s, i)
	}
	backing := &s[0]
	p.Put(s)

	// Next Get reuses the same backing array (length reset to 0, capacity kept).
	s2 := p.Get()
	if len(s2) != 0 {
		t.Errorf("recycled slice should have len 0, got %d", len(s2))
	}
	if cap(s2) < 8 {
		t.Errorf("recycled slice should keep capacity >=8, got %d", cap(s2))
	}
	s2 = s2[:1]
	if &s2[0] != backing {
		t.Errorf("Get should reuse the recycled backing array")
	}

	// Pool empty again -> nil.
	p.Put(s2)
	_ = p.Get()
	if got := p.Get(); got != nil {
		t.Errorf("with the pool empty, Get should return nil")
	}

	// Zero-capacity slices are not pooled.
	p.Put(nil)
	if got := p.Get(); got != nil {
		t.Errorf("Put(nil) must not add a usable buffer")
	}
}

func TestVector_PushBackAndRelease(t *testing.T) {
	var p Pool[string]
	v := New(&p)
	v.PushBack("a")
	v.PushBack("b")
	v.PushBack("c")
	if v.Len() != 3 || v.At(0) != "a" || v.At(2) != "c" {
		t.Fatalf("PushBack/At/Len wrong: len=%d", v.Len())
	}
	if got := v.Slice(); len(got) != 3 || got[1] != "b" {
		t.Errorf("Slice() = %v", got)
	}
	v.Release()

	// The released backing is reused by the next Vector.
	w := New(&p)
	if cap(w.Slice()) < 3 {
		t.Errorf("released capacity should be reused, got cap=%d", cap(w.Slice()))
	}
}
