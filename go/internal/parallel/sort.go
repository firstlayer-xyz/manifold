package parallel

import (
	"runtime"
	"slices"
	"sync"
)

// StableSort is the Go port of manifold::stable_sort from
// src/parallel.h. Sorts slice in-place by the given less function,
// preserving relative order of equal elements.
//
// Par branch: parallel chunked sort + parallel pairwise merge. Each
// worker sorts a contiguous chunk with slices.SortStableFunc; then a
// log-P sequence of pairwise merges combines neighboring chunks
// (each merge level is parallel across the surviving chunk pairs).
// Mirrors the TBB merge_sort tree in src/parallel.h details::mergeRec.
//
// Seq branch is slices.SortStableFunc directly. Below kSeqThreshold this
// is what we want — goroutine fan-out is overhead-only for small n.
//
// We use the generic slices.SortStableFunc rather than sort.SliceStable:
// the latter boxes the slice behind reflect.Value and swaps via a
// reflectlite.Swapper on every move (a measurable cost in the geometry
// sorts), whereas the generic form is reflection-free and is also the
// faithful shape for manifold::stable_sort — a template, not a
// runtime-reflection sort (cf. sortEdgePos in internal/boolean).
func StableSort[T any](policy ExecutionPolicy, slice []T, less func(a, b T) bool) {
	n := len(slice)
	if n <= 1 {
		return
	}
	// Adapt the strict-weak-ordering less to a three-way comparison.
	cmp := func(a, b T) int {
		switch {
		case less(a, b):
			return -1
		case less(b, a):
			return 1
		default:
			return 0
		}
	}
	if policy == Seq {
		slices.SortStableFunc(slice, cmp)
		return
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers

	// Phase 1: parallel sort each chunk.
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
			slices.SortStableFunc(slice[lo:hi], cmp)
		}(s, e)
	}
	wg.Wait()

	// Phase 2: bottom-up pairwise merge. Chunks of size `width` are
	// stably merged into chunks of size 2*width, repeated until the
	// whole slice is one sorted run.
	scratch := make([]T, n)
	width := chunk
	for width < n {
		// Each merge pair becomes one task; run them in parallel.
		var mwg sync.WaitGroup
		for lo := 0; lo < n; lo += 2 * width {
			mid := lo + width
			if mid > n {
				mid = n
			}
			hi := lo + 2*width
			if hi > n {
				hi = n
			}
			if mid == hi {
				continue
			}
			mwg.Add(1)
			go func(lo, mid, hi int) {
				defer mwg.Done()
				stableMerge(slice, scratch, lo, mid, hi, less)
			}(lo, mid, hi)
		}
		mwg.Wait()
		width *= 2
	}
}

// stableMerge writes the stable merge of slice[lo:mid] and
// slice[mid:hi] into scratch[lo:hi], then copies it back into
// slice[lo:hi]. Stability: equal elements retain their original
// relative order (left half wins ties).
func stableMerge[T any](slice, scratch []T, lo, mid, hi int, less func(a, b T) bool) {
	i, j, k := lo, mid, lo
	for i < mid && j < hi {
		if less(slice[j], slice[i]) {
			scratch[k] = slice[j]
			j++
		} else {
			scratch[k] = slice[i]
			i++
		}
		k++
	}
	for i < mid {
		scratch[k] = slice[i]
		i++
		k++
	}
	for j < hi {
		scratch[k] = slice[j]
		j++
		k++
	}
	copy(slice[lo:hi], scratch[lo:hi])
}

// CopyIf is the Go port of manifold::copy_if from src/parallel.h.
// Copies every src[i] for which pred(i) is true into a new slice,
// preserving source order, and returns the new slice.
//
// Par branch is the standard 3-pass parallel filter:
//  1. Each worker counts how many of its chunk pass the predicate.
//  2. Serial exclusive scan over per-chunk counts to compute the
//     write offset for each chunk in the output.
//  3. Each worker writes its kept elements into out[offset..],
//     preserving order within its chunk.
//
// Equivalent in shape to the TBB copy_if approach in the manifold
// codebase. Returns out trimmed to the total kept count.
func CopyIf[T any](policy ExecutionPolicy, src []T, pred func(i int) bool) []T {
	n := len(src)
	if n == 0 {
		return nil
	}
	if policy == Seq {
		out := make([]T, 0, n)
		for i := 0; i < n; i++ {
			if pred(i) {
				out = append(out, src[i])
			}
		}
		return out
	}
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > n {
		nWorkers = n
	}
	chunk := (n + nWorkers - 1) / nWorkers
	counts := make([]int, nWorkers)

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
			c := 0
			for i := lo; i < hi; i++ {
				if pred(i) {
					c++
				}
			}
			counts[idx] = c
		}(w, s, e)
	}
	wg1.Wait()

	offsets := make([]int, nWorkers)
	var running int
	for w := 0; w < nWorkers; w++ {
		offsets[w] = running
		running += counts[w]
	}
	out := make([]T, running)

	var wg2 sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > n {
			e = n
		}
		if s >= e {
			continue
		}
		wg2.Add(1)
		go func(idx, lo, hi int) {
			defer wg2.Done()
			off := offsets[idx]
			for i := lo; i < hi; i++ {
				if pred(i) {
					out[off] = src[i]
					off++
				}
			}
		}(w, s, e)
	}
	wg2.Wait()
	return out
}
