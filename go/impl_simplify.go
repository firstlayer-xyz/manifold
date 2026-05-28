package manifold

import (
	"runtime"
	"sync"

	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// flagStore is the Go port of the anonymous-namespace FlagStore helper in
// src/edge_op.cpp. It collects the indices in [0, n) that satisfy a
// predicate, then invokes a serial action f over them in ascending order.
//
// The predicate test may run in parallel (above the 1e5 threshold), but f
// always runs single-threaded: the edge-collapse / edge-swap operations it
// drives mutate shared mesh state and cannot be parallelized.
//
// Used by CollapseShortEdges, CollapseColinearEdges, and SwapDegenerates.
type flagStore struct {
	s []int
}

// run flags i in [0, n) where pred(i) is true, then calls f(i) for each
// flagged i in ascending order. Mirrors FlagStore::run: parallel pred test
// above 1e5, serial below.
func (fs *flagStore) run(n int, pred func(i int) bool, f func(i int)) {
	if n > 100000 {
		fs.runPar(n, pred, f)
	} else {
		fs.runSeq(n, pred, f)
	}
}

// runSeq mirrors FlagStore::run_seq: collect flagged indices in order, then
// process them in order.
func (fs *flagStore) runSeq(n int, pred func(i int) bool, f func(i int)) {
	fs.s = fs.s[:0]
	for i := 0; i < n; i++ {
		if pred(i) {
			fs.s = append(fs.s, i)
		}
	}
	for _, i := range fs.s {
		f(i)
	}
	fs.s = fs.s[:0]
}

// runPar mirrors FlagStore::run_par: test pred in parallel into per-worker
// buckets (tbb::combinable), concatenate, stable-sort to restore ascending
// order, then call f serially. The Go parallel package has no combinable, so
// the range is chunked across workers — the same pattern the DedupeEdges
// parallel branch uses to mirror a tbb::combinable.
func (fs *flagStore) runPar(n int, pred func(i int) bool, f func(i int)) {
	workers := runtime.GOMAXPROCS(0)
	if workers > n {
		workers = n
	}
	chunk := (n + workers - 1) / workers
	locals := make([][]int, workers)
	var wg sync.WaitGroup
	for w := 0; w < workers; w++ {
		lo := w * chunk
		hi := lo + chunk
		if hi > n {
			hi = n
		}
		if lo >= hi {
			continue
		}
		wg.Add(1)
		go func(w, lo, hi int) {
			defer wg.Done()
			var local []int
			for i := lo; i < hi; i++ {
				if pred(i) {
					local = append(local, i)
				}
			}
			locals[w] = local
		}(w, lo, hi)
	}
	wg.Wait()

	var result []int
	for _, l := range locals {
		result = append(result, l...)
	}
	// C++ stable_sorts the combined buckets (default threshold 1e4); the
	// chunked concatenation above is already ascending, but mirror it.
	parallel.StableSort(parallel.AutoPolicy(len(result), 10000), result, func(a, b int) bool { return a < b })
	for _, i := range result {
		f(i)
	}
}
