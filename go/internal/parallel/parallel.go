// Package parallel is the Go port of the manifold parallel
// primitives from src/parallel.h. It mirrors the C++ public surface
// — ExecutionPolicy, AutoPolicy, ForEachN — used throughout the
// algorithm code in the form
//
//	for_each_n(autoPolicy(n, threshold), countAt(0), n, lambda);
//
// becoming
//
//	parallel.ForEachN(parallel.AutoPolicy(n, threshold), n, func(i int) { ... }).
//
// The C++ implementation dispatches on policy:
//
//   - Seq: a plain serial loop (std::for_each).
//   - Par: tbb::parallel_for over a blocked_range.
//
// We mirror that here: Seq runs in the caller goroutine; Par fans
// out to GOMAXPROCS worker goroutines, each iterating a contiguous
// chunk. The caller is responsible for race-safety of the function
// body, exactly as in C++ — any shared writes must use atomics.
//
// We do NOT port the cancellable variant (ExecutionContext::Impl).
// None of the manifold call sites currently use it, and the
// non-cancellable shim in C++ short-circuits to the simple path,
// so this matches what those call sites actually invoke.
package parallel

import (
	"runtime"
	"sync"
)

// ExecutionPolicy mirrors manifold::ExecutionPolicy from
// src/parallel.h. Two values: Seq (serial) and Par (parallel).
type ExecutionPolicy int

const (
	// Par dispatches to parallel workers.
	Par ExecutionPolicy = iota
	// Seq runs in the caller goroutine.
	Seq
)

// SeqThreshold mirrors manifold::kSeqThreshold = 1e4 from
// src/parallel.h — the default crossover point used by autoPolicy
// when the call site doesn't supply its own threshold.
const SeqThreshold = 10000

// AutoPolicy is the Go port of manifold::autoPolicy(size, threshold)
// from src/parallel.h. Returns Seq when size <= threshold, Par
// otherwise. The threshold defaults to SeqThreshold (matching the
// C++ default-argument behavior) but most call sites supply their
// own — e.g. 1e5 for ReindexVerts, 1e4 for DedupePropVerts.
func AutoPolicy(size int, threshold ...int) ExecutionPolicy {
	t := SeqThreshold
	if len(threshold) > 0 {
		t = threshold[0]
	}
	if size <= t {
		return Seq
	}
	return Par
}

// ForEachN is the Go port of manifold::for_each_n with countAt(0) —
// the counting-iterator form that all of our manifold call sites
// use. Runs f(0), f(1), ..., f(n-1).
//
// Par dispatch fans out to GOMAXPROCS workers, each iterating a
// contiguous index chunk; matches the TBB blocked_range pattern in
// the C++ for_each implementation (src/parallel.h line 401-420).
// Seq dispatch is a plain serial loop, matching std::for_each.
//
// As with C++, f must be safe to invoke concurrently across distinct
// indices in the Par case. Use sync/atomic for any shared writes.
func ForEachN(policy ExecutionPolicy, n int, f func(i int)) {
	if n <= 0 {
		return
	}
	if policy == Seq {
		for i := 0; i < n; i++ {
			f(i)
		}
		return
	}
	// Par.
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		start := w * chunk
		end := start + chunk
		if end > n {
			end = n
		}
		if start >= end {
			continue
		}
		wg.Add(1)
		go func(s, e int) {
			defer wg.Done()
			for i := s; i < e; i++ {
				f(i)
			}
		}(start, end)
	}
	wg.Wait()
}
