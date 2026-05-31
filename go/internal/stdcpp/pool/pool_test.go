package pool

import "testing"

func TestFreeList_RecyclesAndZeroes(t *testing.T) {
	type node struct {
		val   int
		child *node
	}
	var fl FreeList[node]

	// Get on an empty list allocates a fresh, zeroed node.
	a := fl.Get()
	if a.val != 0 || a.child != nil {
		t.Fatalf("Get on empty list should return zeroed node, got %+v", *a)
	}

	// Populate and return it; Put must zero (so the pool pins no stale child ref).
	a.val = 7
	a.child = &node{val: 99}
	fl.Put(a)
	if a.val != 0 || a.child != nil {
		t.Errorf("Put should zero the node; got %+v", *a)
	}

	// Next Get reuses the same recycled (zeroed) node.
	b := fl.Get()
	if b != a {
		t.Errorf("Get should reuse the recycled node")
	}
	if b.val != 0 || b.child != nil {
		t.Errorf("recycled node should be zeroed, got %+v", *b)
	}

	// Pool is empty again → fresh allocation.
	c := fl.Get()
	if c == a {
		t.Errorf("with the pool empty, Get should allocate a new node")
	}
}

func TestFreeList_LIFOReuse(t *testing.T) {
	var fl FreeList[int]
	p1, p2 := fl.Get(), fl.Get()
	fl.Put(p1)
	fl.Put(p2)
	// LIFO: last Put (p2) is returned first.
	if got := fl.Get(); got != p2 {
		t.Errorf("expected LIFO reuse of p2")
	}
	if got := fl.Get(); got != p1 {
		t.Errorf("expected p1 next")
	}
}
