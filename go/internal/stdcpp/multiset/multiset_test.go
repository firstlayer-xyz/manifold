package multiset

import (
	"math/rand"
	"testing"
)

type kv struct {
	k  float64
	id int
}

func ascK(a, b kv) bool { return a.k < b.k }

// TestMultiset_EqualKeyFIFO: equal keys must pop earliest-inserted first —
// the std::multiset tie-break that gates byte-identical triangulation.
func TestMultiset_EqualKeyFIFO(t *testing.T) {
	ms := New[kv](ascK)
	for id := 0; id < 6; id++ {
		ms.Insert(kv{k: 1.0, id: id})
	}
	for want := 0; want < 6; want++ {
		h, ok := ms.Begin()
		if !ok {
			t.Fatalf("empty after %d pops", want)
		}
		if h.Value().id != want {
			t.Fatalf("pop %d: got id %d, want %d (FIFO)", want, h.Value().id, want)
		}
		ms.Erase(h)
	}
	if ms.Len() != 0 {
		t.Fatalf("Len = %d, want 0", ms.Len())
	}
}

// TestMultiset_OrderedPop: ascending key, then FIFO among equal keys.
func TestMultiset_OrderedPop(t *testing.T) {
	ms := New[kv](ascK)
	ms.Insert(kv{3, 0})
	ms.Insert(kv{1, 1})
	ms.Insert(kv{2, 2})
	ms.Insert(kv{1, 3}) // equal to id 1, inserted later
	ms.Insert(kv{2, 4})
	wantIDs := []int{1, 3, 2, 4, 0} // k=1(id1,id3), k=2(id2,id4), k=3(id0)
	for _, want := range wantIDs {
		h, _ := ms.Begin()
		if h.Value().id != want {
			t.Fatalf("got id %d, want %d", h.Value().id, want)
		}
		ms.Erase(h)
	}
}

// TestMultiset_HandleStability: a handle erases the right element even after
// other inserts/erases shuffle the tree.
func TestMultiset_HandleStability(t *testing.T) {
	ms := New[kv](ascK)
	ms.Insert(kv{1, 0})
	hB := ms.Insert(kv{1, 1})
	ms.Insert(kv{1, 2})
	hX := ms.Insert(kv{0, 9})
	ms.Erase(hX)
	ms.Insert(kv{5, 8})
	ms.Erase(hB) // erase the middle equal-key element by its old handle
	for _, want := range []int{0, 2, 8} {
		h, ok := ms.Begin()
		if !ok {
			t.Fatalf("empty, expected id %d", want)
		}
		if h.Value().id != want {
			t.Fatalf("got id %d, want %d", h.Value().id, want)
		}
		ms.Erase(h)
	}
}

// TestMultiset_InOrderDescending exercises the holes_ comparator (pos.x
// descending) and the InOrder iteration used to walk holes.
func TestMultiset_InOrderDescending(t *testing.T) {
	ms := New[kv](func(a, b kv) bool { return a.k > b.k }) // MaxX
	for _, k := range []float64{2, 5, 1, 5, 3} {
		ms.Insert(kv{k: k})
	}
	got := ms.InOrder()
	wantK := []float64{5, 5, 3, 2, 1}
	if len(got) != len(wantK) {
		t.Fatalf("InOrder len %d, want %d", len(got), len(wantK))
	}
	for i, k := range wantK {
		if got[i].k != k {
			t.Fatalf("InOrder[%d].k = %v, want %v", i, got[i].k, k)
		}
	}
}

// TestMultiset_FuzzAgainstReference hammers random insert/erase/min against a
// brute-force reference (min by key then insertion seq) over many ops with
// many equal-key collisions — validating the AVL delete + FIFO tie-break.
func TestMultiset_FuzzAgainstReference(t *testing.T) {
	rng := rand.New(rand.NewSource(0xC0FFEE))
	ms := New[kv](ascK)
	type live struct {
		h Handle[kv]
		v kv
	}
	var lives []live
	for op := 0; op < 50000; op++ {
		if len(lives) == 0 || rng.Intn(100) < 55 {
			// Insert with few distinct keys to force many equal-key ties.
			v := kv{k: float64(rng.Intn(8)), id: op}
			h := ms.Insert(v)
			lives = append(lives, live{h, v})
		} else {
			i := rng.Intn(len(lives))
			ms.Erase(lives[i].h)
			lives[i] = lives[len(lives)-1]
			lives = lives[:len(lives)-1]
		}
		if ms.Len() != len(lives) {
			t.Fatalf("op %d: Len %d != ref %d", op, ms.Len(), len(lives))
		}
		if len(lives) == 0 {
			continue
		}
		// Reference min: smallest k, then smallest insertion seq.
		best := lives[0]
		for _, l := range lives[1:] {
			if l.v.k < best.v.k || (l.v.k == best.v.k && l.h.seq < best.h.seq) {
				best = l
			}
		}
		h, ok := ms.Begin()
		if !ok {
			t.Fatalf("op %d: Begin empty but %d live", op, len(lives))
		}
		if h.seq != best.h.seq {
			t.Fatalf("op %d: Begin seq=%d (k=%v) != ref seq=%d (k=%v)",
				op, h.seq, h.val.k, best.h.seq, best.v.k)
		}
	}
}
