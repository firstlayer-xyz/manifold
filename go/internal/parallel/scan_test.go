package parallel

import "testing"

// TestExclusiveScanFunc covers the binary-op form (used by Boolean3::Result
// with AbsSum) against hand-computed exclusive scans.
func TestExclusiveScanFunc(t *testing.T) {
	absSum := func(a, b int) int {
		if a < 0 {
			a = -a
		}
		if b < 0 {
			b = -b
		}
		return a + b
	}
	in := []int{3, -2, 5}
	out := make([]int, len(in))
	ExclusiveScanFunc(Seq, in, out, 0, absSum)
	if got, want := out, []int{0, 3, 5}; !intsEqual(got, want) {
		t.Errorf("AbsSum init 0 = %v, want %v", got, want)
	}
	ExclusiveScanFunc(Seq, in, out, 10, absSum)
	if got, want := out, []int{10, 13, 15}; !intsEqual(got, want) {
		t.Errorf("AbsSum init 10 = %v, want %v", got, want)
	}
	// Plain + op (generality check).
	plus := func(a, b int) int { return a + b }
	in2 := []int{1, 2, 3}
	out2 := make([]int, len(in2))
	ExclusiveScanFunc(Seq, in2, out2, 0, plus)
	if got, want := out2, []int{0, 1, 3}; !intsEqual(got, want) {
		t.Errorf("plus = %v, want %v", got, want)
	}
}

func intsEqual(a, b []int) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
