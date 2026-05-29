package disjointsets

import "testing"

// TestDisjointSets_Basic covers the find/unite/same primitives on a
// small forest.
func TestDisjointSets_Basic(t *testing.T) {
	d := New(6)
	if d.Size() != 6 {
		t.Fatalf("Size: got %d, want 6", d.Size())
	}
	// Each element is its own representative initially.
	for i := 0; i < 6; i++ {
		if d.Find(i) != i {
			t.Errorf("Find(%d) = %d, want %d", i, d.Find(i), i)
		}
	}
	if d.Same(0, 1) {
		t.Error("Same(0,1) before Unite should be false")
	}
	d.Unite(0, 1)
	if !d.Same(0, 1) {
		t.Error("Same(0,1) after Unite should be true")
	}
	d.Unite(2, 3)
	d.Unite(1, 2) // merges {0,1} with {2,3}
	for _, pair := range [][2]int{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}} {
		if !d.Same(pair[0], pair[1]) {
			t.Errorf("Same(%d, %d) should be true after chained Unite", pair[0], pair[1])
		}
	}
	if d.Same(0, 4) {
		t.Error("Same(0, 4) should be false; 4 was never united")
	}
}

// TestDisjointSets_ConnectedComponents matches the C++ semantics:
// lonely (rank-0) nodes get fresh sequential labels; multi-element
// components share a label per root.
func TestDisjointSets_ConnectedComponents(t *testing.T) {
	d := New(5)
	d.Unite(0, 1)
	d.Unite(1, 2)
	// {0,1,2} is one component; {3}, {4} are lonely.
	out := make([]int32, 5)
	got := d.ConnectedComponents(out)
	if got != 3 {
		t.Fatalf("ConnectedComponents count: got %d, want 3", got)
	}
	// Verify 0/1/2 share a label and 3, 4 each have unique labels.
	if out[0] != out[1] || out[1] != out[2] {
		t.Errorf("0/1/2 should share a label, got %v", out)
	}
	if out[3] == out[0] || out[4] == out[0] {
		t.Errorf("3 and 4 should not share label with 0/1/2, got %v", out)
	}
	if out[3] == out[4] {
		t.Errorf("lonely nodes 3 and 4 should have distinct labels, got %v", out)
	}
}
