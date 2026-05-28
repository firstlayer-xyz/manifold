package manifold

import (
	"testing"
)

// TestFlagStore_Seq exercises the serial branch (n <= 1e5): flagged
// indices must be processed in ascending order, exactly once each.
func TestFlagStore_Seq(t *testing.T) {
	const n = 1000
	var fs flagStore
	var got []int
	fs.run(n, func(i int) bool { return i%3 == 0 }, func(i int) { got = append(got, i) })

	var want []int
	for i := 0; i < n; i++ {
		if i%3 == 0 {
			want = append(want, i)
		}
	}
	if len(got) != len(want) {
		t.Fatalf("seq: got %d indices, want %d", len(got), len(want))
	}
	for k := range want {
		if got[k] != want[k] {
			t.Fatalf("seq: index %d = %d, want %d (order must be ascending)", k, got[k], want[k])
		}
	}
}

// TestFlagStore_Par exercises the parallel branch (n > 1e5). The predicate
// runs concurrently, but f must still see ascending order, once each.
func TestFlagStore_Par(t *testing.T) {
	const n = 250000 // > 1e5 -> runPar
	var fs flagStore
	var got []int
	// f runs serially per the FlagStore contract, so no lock is needed.
	fs.run(n, func(i int) bool { return i%7 == 0 }, func(i int) { got = append(got, i) })

	var want []int
	for i := 0; i < n; i++ {
		if i%7 == 0 {
			want = append(want, i)
		}
	}
	if len(got) != len(want) {
		t.Fatalf("par: got %d indices, want %d", len(got), len(want))
	}
	for k := range want {
		if got[k] != want[k] {
			t.Fatalf("par: index %d = %d, want %d (must be ascending after merge+sort)", k, got[k], want[k])
		}
	}
}

// TestFlagStore_Reuse verifies the store resets between runs.
func TestFlagStore_Reuse(t *testing.T) {
	var fs flagStore
	var first []int
	fs.run(10, func(i int) bool { return i < 3 }, func(i int) { first = append(first, i) })
	var second []int
	fs.run(10, func(i int) bool { return i >= 8 }, func(i int) { second = append(second, i) })

	if len(first) != 3 || first[0] != 0 || first[2] != 2 {
		t.Fatalf("first run = %v, want [0 1 2]", first)
	}
	if len(second) != 2 || second[0] != 8 || second[1] != 9 {
		t.Fatalf("second run = %v, want [8 9]", second)
	}
}
