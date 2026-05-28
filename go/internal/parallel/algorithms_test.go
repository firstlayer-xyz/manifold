package parallel

import (
	"sort"
	"sync/atomic"
	"testing"
)

func TestReduce_SeqMatchesPar(t *testing.T) {
	in := make([]int, 1<<14)
	for i := range in {
		in[i] = i
	}
	add := func(a, b int) int { return a + b }
	want := Reduce(Seq, in, 0, add)
	got := Reduce(Par, in, 0, add)
	if got != want {
		t.Errorf("reduce: par=%d seq=%d", got, want)
	}
	if want != (len(in)-1)*len(in)/2 {
		t.Errorf("reduce result wrong: got %d", want)
	}
}

func TestTransformReduce_SeqMatchesPar(t *testing.T) {
	in := make([]int, 1<<14)
	for i := range in {
		in[i] = i
	}
	sq := func(x int) int { return x * x }
	add := func(a, b int) int { return a + b }
	want := TransformReduce(Seq, in, 0, add, sq)
	got := TransformReduce(Par, in, 0, add, sq)
	if got != want {
		t.Errorf("transform_reduce: par=%d seq=%d", got, want)
	}
}

func TestAllOf_True(t *testing.T) {
	n := 1 << 14
	pred := func(i int) bool { return true }
	if !AllOf(Par, n, pred) {
		t.Error("AllOf: par should be true")
	}
	if !AllOf(Seq, n, pred) {
		t.Error("AllOf: seq should be true")
	}
}

func TestAllOf_OneFalse(t *testing.T) {
	n := 1 << 14
	bad := n / 2
	pred := func(i int) bool { return i != bad }
	if AllOf(Par, n, pred) {
		t.Error("AllOf: par should be false")
	}
	if AllOf(Seq, n, pred) {
		t.Error("AllOf: seq should be false")
	}
}

func TestAnyOf(t *testing.T) {
	n := 1 << 14
	hit := n / 3
	pred := func(i int) bool { return i == hit }
	if !AnyOf(Par, n, pred) {
		t.Error("AnyOf: par should be true")
	}
	if !AnyOf(Seq, n, pred) {
		t.Error("AnyOf: seq should be true")
	}
	if AnyOf(Par, n, func(int) bool { return false }) {
		t.Error("AnyOf: par should be false when no match")
	}
}

func TestCountIf(t *testing.T) {
	n := 1 << 14
	pred := func(i int) bool { return i%3 == 0 }
	want := 0
	for i := 0; i < n; i++ {
		if pred(i) {
			want++
		}
	}
	if got := CountIf(Par, n, pred); got != want {
		t.Errorf("CountIf par: got %d want %d", got, want)
	}
	if got := CountIf(Seq, n, pred); got != want {
		t.Errorf("CountIf seq: got %d want %d", got, want)
	}
}

func TestTransform(t *testing.T) {
	n := 1 << 13
	in := make([]int, n)
	for i := range in {
		in[i] = i
	}
	out := make([]int, n)
	Transform(Par, in, out, func(s int) int { return s * 2 })
	for i, v := range out {
		if v != 2*i {
			t.Errorf("Transform[%d]: got %d want %d", i, v, 2*i)
			break
		}
	}
}

func TestFill(t *testing.T) {
	s := make([]int32, 5000)
	Fill(Par, s, int32(7))
	for i, v := range s {
		if v != 7 {
			t.Errorf("Fill[%d]: %d", i, v)
			break
		}
	}
}

func TestSequence(t *testing.T) {
	s := make([]int32, 5000)
	Sequence(Par, s)
	for i, v := range s {
		if v != int32(i) {
			t.Errorf("Sequence[%d]: %d", i, v)
			break
		}
	}
}

func TestGather(t *testing.T) {
	n := 100
	src := make([]int, n)
	for i := range src {
		src[i] = i * 10
	}
	idx := make([]int32, n)
	for i := range idx {
		idx[i] = int32(n - 1 - i)
	}
	out := make([]int, n)
	Gather(Par, idx, src, out)
	for i := 0; i < n; i++ {
		if out[i] != (n-1-i)*10 {
			t.Errorf("Gather[%d]: got %d", i, out[i])
		}
	}
}

func TestScatter(t *testing.T) {
	n := 100
	src := make([]int, n)
	for i := range src {
		src[i] = i
	}
	idx := make([]int32, n)
	for i := range idx {
		idx[i] = int32(n - 1 - i)
	}
	out := make([]int, n)
	Scatter(Par, src, idx, out)
	for i := 0; i < n; i++ {
		if out[i] != n-1-i {
			t.Errorf("Scatter[%d]: %d", i, out[i])
		}
	}
}

func TestInclusiveScan(t *testing.T) {
	n := 1 << 14
	in := make([]int32, n)
	for i := range in {
		in[i] = int32(i + 1)
	}
	wantOut := make([]int32, n)
	InclusiveScan(Seq, in, wantOut)
	gotOut := make([]int32, n)
	InclusiveScan(Par, in, gotOut)
	for i := 0; i < n; i++ {
		if wantOut[i] != gotOut[i] {
			t.Errorf("InclusiveScan[%d]: par=%d seq=%d", i, gotOut[i], wantOut[i])
			break
		}
	}
	if wantOut[n-1] != int32(n*(n+1)/2) {
		t.Errorf("InclusiveScan last: got %d", wantOut[n-1])
	}
}

func TestExclusiveScan(t *testing.T) {
	n := 1 << 14
	in := make([]int32, n)
	for i := range in {
		in[i] = int32(i + 1)
	}
	wantOut := make([]int32, n)
	ExclusiveScan(Seq, in, wantOut, int32(0))
	gotOut := make([]int32, n)
	ExclusiveScan(Par, in, gotOut, int32(0))
	for i := 0; i < n; i++ {
		if wantOut[i] != gotOut[i] {
			t.Errorf("ExclusiveScan[%d]: par=%d seq=%d", i, gotOut[i], wantOut[i])
			break
		}
	}
}

func TestStableSort(t *testing.T) {
	type kv struct{ k, v int }
	n := 5000
	mk := func() []kv {
		out := make([]kv, n)
		for i := range out {
			out[i] = kv{k: i % 17, v: i}
		}
		return out
	}
	less := func(a, b kv) bool { return a.k < b.k }
	seqOut := mk()
	parOut := mk()
	StableSort(Seq, seqOut, less)
	StableSort(Par, parOut, less)
	for i, e := range parOut {
		if e != seqOut[i] {
			t.Errorf("StableSort[%d]: par=%v seq=%v", i, e, seqOut[i])
			break
		}
	}
	// Verify ordering: keys must be non-decreasing.
	if !sort.SliceIsSorted(parOut, func(i, j int) bool { return parOut[i].k < parOut[j].k }) {
		t.Error("StableSort: par output not sorted by key")
	}
	// Verify stability: within each key group, v is non-decreasing.
	for i := 1; i < len(parOut); i++ {
		if parOut[i].k == parOut[i-1].k && parOut[i].v < parOut[i-1].v {
			t.Errorf("StableSort: not stable at i=%d", i)
			break
		}
	}
}

func TestCopyIf(t *testing.T) {
	n := 1 << 13
	src := make([]int, n)
	for i := range src {
		src[i] = i
	}
	pred := func(i int) bool { return src[i]%3 == 0 }
	seqOut := CopyIf(Seq, src, pred)
	parOut := CopyIf(Par, src, pred)
	if len(seqOut) != len(parOut) {
		t.Fatalf("CopyIf len: seq=%d par=%d", len(seqOut), len(parOut))
	}
	for i, v := range parOut {
		if v != seqOut[i] {
			t.Errorf("CopyIf[%d]: par=%d seq=%d", i, v, seqOut[i])
			break
		}
	}
}

func TestParRaceFree(t *testing.T) {
	// Smoke test: distinct slot writes under Par must be data-race-free
	// (run with -race for it to mean anything).
	n := 50000
	out := make([]int64, n)
	ForEachN(Par, n, func(i int) {
		atomic.StoreInt64(&out[i], int64(i))
	})
	for i, v := range out {
		if v != int64(i) {
			t.Errorf("[%d]: %d", i, v)
			break
		}
	}
}
