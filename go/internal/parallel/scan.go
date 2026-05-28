package parallel

import (
	"runtime"
	"sync"
)

// Numeric is the constraint for scan element types — every type the
// manifold scan call sites actually use (int32 in CompactProps,
// int for keep arrays). Mirrors the implicit T in C++ inclusive_scan
// and exclusive_scan.
type Numeric interface {
	~int | ~int32 | ~int64 | ~uint32 | ~uint64
}

// InclusiveScan is the Go port of manifold::inclusive_scan from
// src/parallel.h with the default-summation form (plus, init 0).
// Writes out[i] = sum(in[0..=i]) for every i, with len(out) >= len(in).
//
// Par branch uses the standard 2-pass parallel prefix sum:
//
//   - Pass 1 (parallel): each worker computes the sum of its chunk.
//   - Pass 2 (serial):   exclusive prefix sum over chunk totals.
//   - Pass 3 (parallel): each worker fills out[i] = chunkOffset +
//     running sum within its chunk.
//
// Matches the work-efficient parallel scan TBB uses in
// tbb::parallel_scan. Two synchronization barriers, O(n / P + P)
// span.
func InclusiveScan[T Numeric](policy ExecutionPolicy, in []T, out []T) {
	n := len(in)
	if n == 0 {
		return
	}
	if policy == Seq {
		var acc T
		for i := 0; i < n; i++ {
			acc += in[i]
			out[i] = acc
		}
		return
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	chunkSums := make([]T, nWorkers)

	// Pass 1: per-chunk sums.
	var wg1 sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg1.Add(1)
		go func(idx, lo, hi int) {
			defer wg1.Done()
			var sum T
			for i := lo; i < hi; i++ {
				sum += in[i]
			}
			chunkSums[idx] = sum
		}(w, s, e)
	}
	wg1.Wait()

	// Pass 2: serial exclusive scan over chunkSums. After this,
	// chunkOffsets[w] = sum of all chunkSums[0..w-1].
	chunkOffsets := make([]T, nWorkers)
	var running T
	for w := 0; w < nWorkers; w++ {
		chunkOffsets[w] = running
		running += chunkSums[w]
	}

	// Pass 3: each worker fills its slice of out with offset + running.
	var wg3 sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg3.Add(1)
		go func(idx, lo, hi int) {
			defer wg3.Done()
			acc := chunkOffsets[idx]
			for i := lo; i < hi; i++ {
				acc += in[i]
				out[i] = acc
			}
		}(w, s, e)
	}
	wg3.Wait()
}

// ExclusiveScan is the Go port of manifold::exclusive_scan from
// src/parallel.h. Writes out[0] = init, out[i] = init + sum(in[0..i-1])
// for i >= 1. Same two-pass parallel structure as InclusiveScan.
func ExclusiveScan[T Numeric](policy ExecutionPolicy, in []T, out []T, init T) {
	n := len(in)
	if n == 0 {
		return
	}
	if policy == Seq {
		acc := init
		for i := 0; i < n; i++ {
			next := acc + in[i]
			out[i] = acc
			acc = next
		}
		return
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	chunkSums := make([]T, nWorkers)

	var wg1 sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg1.Add(1)
		go func(idx, lo, hi int) {
			defer wg1.Done()
			var sum T
			for i := lo; i < hi; i++ {
				sum += in[i]
			}
			chunkSums[idx] = sum
		}(w, s, e)
	}
	wg1.Wait()

	chunkOffsets := make([]T, nWorkers)
	running := init
	for w := 0; w < nWorkers; w++ {
		chunkOffsets[w] = running
		running += chunkSums[w]
	}

	var wg3 sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg3.Add(1)
		go func(idx, lo, hi int) {
			defer wg3.Done()
			acc := chunkOffsets[idx]
			for i := lo; i < hi; i++ {
				next := acc + in[i]
				out[i] = acc
				acc = next
			}
		}(w, s, e)
	}
	wg3.Wait()
}
