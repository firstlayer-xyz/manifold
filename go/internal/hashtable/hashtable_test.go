package hashtable

import (
	"sync"
	"testing"
)

func TestHashTable_BasicInsertLookup(t *testing.T) {
	h := New[int](16, 1)
	h.Insert(42, 7)
	h.Insert(1234, -3)
	if v, ok := h.Lookup(42); !ok || v != 7 {
		t.Errorf("Lookup(42) = (%v, %v), want (7, true)", v, ok)
	}
	if v, ok := h.Lookup(1234); !ok || v != -3 {
		t.Errorf("Lookup(1234) = (%v, %v), want (-3, true)", v, ok)
	}
	if v, ok := h.Lookup(0); ok {
		t.Errorf("Lookup(0) = (%v, true), want (_, false)", v)
	}
}

func TestHashTable_DuplicateInsertNoOp(t *testing.T) {
	h := New[int](16, 1)
	h.Insert(42, 7)
	h.Insert(42, 99) // duplicate key — must be silently ignored, value unchanged
	if v, ok := h.Lookup(42); !ok || v != 7 {
		t.Errorf("after duplicate insert: Lookup(42) = (%v, %v), want (7, true)", v, ok)
	}
	if h.Entries() != 1 {
		t.Errorf("Entries = %d, want 1 (no duplicate)", h.Entries())
	}
}

func TestHashTable_FullDropsInserts(t *testing.T) {
	// C++ Full() = used*2 > size. Capacity=8; full when used >= 5.
	h := New[int](8, 1)
	for i := 0; i < 5; i++ {
		h.Insert(uint64(i+1), i*10)
	}
	if !h.Full() {
		t.Fatalf("HashTable should be Full at used=5/cap=8 (5*2=10 > 8); got Entries=%d", h.Entries())
	}
	h.Insert(uint64(100), 999) // must be silently dropped
	if _, ok := h.Lookup(100); ok {
		t.Error("insert past Full should be silently dropped")
	}
}

func TestHashTable_ConcurrentInsert(t *testing.T) {
	const numKeys = 5000
	h := New[int](numKeys*4, 1) // overprovision to stay below Full
	var wg sync.WaitGroup
	const workers = 8
	chunk := numKeys / workers
	for w := 0; w < workers; w++ {
		s := w * chunk
		e := s + chunk
		if e > numKeys {
			e = numKeys
		}
		wg.Add(1)
		go func(s, e int) {
			defer wg.Done()
			for i := s; i < e; i++ {
				h.Insert(uint64(i+1), i)
			}
		}(s, e)
	}
	wg.Wait()
	if got, want := h.Entries(), numKeys; got != want {
		t.Errorf("Entries after concurrent insert: %d, want %d", got, want)
	}
	for i := 0; i < numKeys; i++ {
		if v, ok := h.Lookup(uint64(i + 1)); !ok || v != i {
			t.Errorf("Lookup(%d) = (%v, %v), want (%d, true)", i+1, v, ok, i)
			break
		}
	}
}

func TestHashTable_AtInPlaceMutation(t *testing.T) {
	h := New[int](16, 1)
	h.Insert(42, 7)
	for i := 0; i < h.Size(); i++ {
		if h.KeyAt(i) == 42 {
			*h.At(i) = 100
			break
		}
	}
	if v, ok := h.Lookup(42); !ok || v != 100 {
		t.Errorf("after At() mutation: Lookup(42) = (%v, %v), want (100, true)", v, ok)
	}
}

func TestHashTable_KeyAtOpen(t *testing.T) {
	h := New[int](16, 1)
	// Empty table — every slot reads as Open.
	for i := 0; i < h.Size(); i++ {
		if h.KeyAt(i) != Open {
			t.Errorf("KeyAt(%d) on empty = %v, want Open (max uint64)", i, h.KeyAt(i))
		}
	}
}
