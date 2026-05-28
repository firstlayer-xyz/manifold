package parallel

import (
	"runtime"
	"sync"
	"sync/atomic"
)

// Reduce is the Go port of manifold::reduce from src/parallel.h.
// Combines every element of slice (folded with f against init) into a
// single value. f must be associative and commutative — the Par
// branch reorders combinations, matching the C++ "non-deterministic
// otherwise" caveat.
//
// Seq branch is the plain serial fold (mirrors std::reduce). Par
// branch chunks the slice across GOMAXPROCS workers, each worker
// computes a partial fold against init, then the final fold combines
// the worker partials serially. This is the same shape as TBB's
// parallel_reduce in src/parallel.h reduce().
func Reduce[T any](policy ExecutionPolicy, slice []T, init T, f func(a, b T) T) T {
	if len(slice) == 0 {
		return init
	}
	if policy == Seq {
		acc := init
		for _, v := range slice {
			acc = f(acc, v)
		}
		return acc
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > len(slice) {
		nWorkers = len(slice)
	}
	chunk := (len(slice) + nWorkers - 1) / nWorkers
	partials := make([]T, nWorkers)
	for w := 0; w < nWorkers; w++ {
		partials[w] = init
	}
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > len(slice) {
			e = len(slice)
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(idx, lo, hi int) {
			defer wg.Done()
			acc := init
			for i := lo; i < hi; i++ {
				acc = f(acc, slice[i])
			}
			partials[idx] = acc
		}(w, s, e)
	}
	wg.Wait()
	result := init
	for _, p := range partials {
		result = f(result, p)
	}
	return result
}

// TransformReduce is the Go port of manifold::transform_reduce from
// src/parallel.h. Applies g to every element, then combines the
// results with f against init. Same associativity/commutativity
// caveat as Reduce.
func TransformReduce[S any, T any](policy ExecutionPolicy, slice []S, init T,
	f func(a, b T) T, g func(s S) T) T {
	if len(slice) == 0 {
		return init
	}
	if policy == Seq {
		acc := init
		for _, v := range slice {
			acc = f(acc, g(v))
		}
		return acc
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > len(slice) {
		nWorkers = len(slice)
	}
	chunk := (len(slice) + nWorkers - 1) / nWorkers
	partials := make([]T, nWorkers)
	for w := 0; w < nWorkers; w++ {
		partials[w] = init
	}
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > len(slice) {
			e = len(slice)
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(idx, lo, hi int) {
			defer wg.Done()
			acc := init
			for i := lo; i < hi; i++ {
				acc = f(acc, g(slice[i]))
			}
			partials[idx] = acc
		}(w, s, e)
	}
	wg.Wait()
	result := init
	for _, p := range partials {
		result = f(result, p)
	}
	return result
}

// AllOf is the Go port of manifold::all_of from src/parallel.h.
// Returns true iff pred(i) is true for every i in [0, n). Seq path
// short-circuits at the first false. Par path runs predicates on
// every i in parallel; an atomic flag signals "found false" so other
// workers can exit early, mirroring the C++ tbb cancellation.
func AllOf(policy ExecutionPolicy, n int, pred func(i int) bool) bool {
	if n <= 0 {
		return true
	}
	if policy == Seq {
		for i := 0; i < n; i++ {
			if !pred(i) {
				return false
			}
		}
		return true
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	var stop atomic.Bool
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(lo, hi int) {
			defer wg.Done()
			for i := lo; i < hi; i++ {
				if stop.Load() {
					return
				}
				if !pred(i) {
					stop.Store(true)
					return
				}
			}
		}(s, e)
	}
	wg.Wait()
	return !stop.Load()
}

// AnyOf is the Go port of manifold::any_of. True iff pred(i) is true
// for some i. Symmetric to AllOf.
func AnyOf(policy ExecutionPolicy, n int, pred func(i int) bool) bool {
	if n <= 0 {
		return false
	}
	if policy == Seq {
		for i := 0; i < n; i++ {
			if pred(i) {
				return true
			}
		}
		return false
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	var stop atomic.Bool
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(lo, hi int) {
			defer wg.Done()
			for i := lo; i < hi; i++ {
				if stop.Load() {
					return
				}
				if pred(i) {
					stop.Store(true)
					return
				}
			}
		}(s, e)
	}
	wg.Wait()
	return stop.Load()
}

// CountIf is the Go port of manifold::count_if. Returns the number
// of i in [0, n) for which pred(i) is true. Par accumulates per-
// worker partial counts, then sums them, matching tbb::parallel_reduce
// with an int accumulator.
func CountIf(policy ExecutionPolicy, n int, pred func(i int) bool) int {
	if n <= 0 {
		return 0
	}
	if policy == Seq {
		count := 0
		for i := 0; i < n; i++ {
			if pred(i) {
				count++
			}
		}
		return count
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	partials := make([]int, nWorkers)
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(idx, lo, hi int) {
			defer wg.Done()
			c := 0
			for i := lo; i < hi; i++ {
				if pred(i) {
					c++
				}
			}
			partials[idx] = c
		}(w, s, e)
	}
	wg.Wait()
	total := 0
	for _, p := range partials {
		total += p
	}
	return total
}
