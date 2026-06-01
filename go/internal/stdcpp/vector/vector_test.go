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

func TestNestedPool_RecyclesInnerBackings(t *testing.T) {
	var p NestedPool[int]

	// Empty pool: Get returns nil.
	if s := p.Get(); s != nil {
		t.Fatalf("Get on empty pool should return nil")
	}

	// Build a nested vector with two inner slices, then return it.
	s := PushInner[int](nil)
	s[0] = append(s[0], 1, 2, 3)
	inner0 := &s[0][0]
	s = PushInner(s)
	s[1] = append(s[1], 4, 5)
	p.Put(s)

	// The next Get reuses the outer (len 0) and PushInner reuses the inner
	// backing (reset to length 0).
	s2 := p.Get()
	if len(s2) != 0 {
		t.Errorf("recycled outer should have len 0, got %d", len(s2))
	}
	s2 = PushInner(s2)
	if len(s2[0]) != 0 {
		t.Errorf("re-exposed inner should be reset to len 0, got %d", len(s2[0]))
	}
	s2[0] = append(s2[0], 9)
	if &s2[0][0] != inner0 {
		t.Errorf("PushInner should reuse the recycled inner backing array")
	}
}

// TestPushInner_ResetsStaleInnerBeyondPrevLen guards the subtle case: a buffer
// that once held 3 inners, reused for a cycle using only 1, must reset inner #3
// when a later cycle re-grows to 3 -- otherwise stale data leaks through.
func TestPushInner_ResetsStaleInnerBeyondPrevLen(t *testing.T) {
	var s [][]int
	for i := 0; i < 3; i++ {
		s = PushInner(s)
		s[i] = append(s[i], 100+i)
	}
	s = s[:0] // Reset, as NestedPool.Put/Get would leave it.

	s = PushInner(s) // single-inner cycle -- resets index 0 only
	s[0] = append(s[0], 7)
	s = s[:0]

	s = PushInner(s) // 0
	s = PushInner(s) // 1
	s = PushInner(s) // 2 -- was never reset by the len-1 cycle
	if len(s[2]) != 0 {
		t.Errorf("PushInner must reset a re-exposed inner beyond prev len, got len %d", len(s[2]))
	}
}
