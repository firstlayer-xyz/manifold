package parallel

import (
	"sync/atomic"
	"testing"
)

// TestAutoPolicy_Threshold verifies the default and explicit
// threshold paths mirror src/parallel.h's autoPolicy.
func TestAutoPolicy_Threshold(t *testing.T) {
	if AutoPolicy(SeqThreshold) != Seq {
		t.Errorf("size == default threshold should be Seq")
	}
	if AutoPolicy(SeqThreshold+1) != Par {
		t.Errorf("size > default threshold should be Par")
	}
	if AutoPolicy(50, 100) != Seq {
		t.Errorf("size below custom threshold should be Seq")
	}
	if AutoPolicy(150, 100) != Par {
		t.Errorf("size above custom threshold should be Par")
	}
}

// TestForEachN_Seq covers every index exactly once in serial mode.
func TestForEachN_Seq(t *testing.T) {
	n := 100
	hit := make([]int, n)
	ForEachN(Seq, n, func(i int) { hit[i]++ })
	for i, h := range hit {
		if h != 1 {
			t.Errorf("seq: index %d hit %d times", i, h)
		}
	}
}

// TestForEachN_Par verifies parallel dispatch covers every index
// exactly once. Uses atomic adds because the workers race on hit[i]
// only via distinct i — we use a global counter to also confirm the
// total count.
func TestForEachN_Par(t *testing.T) {
	n := 50000
	hit := make([]int32, n)
	var total int32
	ForEachN(Par, n, func(i int) {
		atomic.AddInt32(&hit[i], 1)
		atomic.AddInt32(&total, 1)
	})
	if total != int32(n) {
		t.Errorf("par: total = %d, want %d", total, n)
	}
	for i, h := range hit {
		if h != 1 {
			t.Errorf("par: index %d hit %d times", i, h)
		}
	}
}

// TestForEachN_Empty: n == 0 must not crash.
func TestForEachN_Empty(t *testing.T) {
	called := false
	ForEachN(Par, 0, func(int) { called = true })
	ForEachN(Seq, 0, func(int) { called = true })
	if called {
		t.Errorf("ForEachN with n=0 must not invoke f")
	}
}
