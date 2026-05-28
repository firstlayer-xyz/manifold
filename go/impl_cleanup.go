package manifold

import (
	"math"
	"runtime"
	"sync"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// SplitPinchedVerts is the Go port of C++
// Manifold::Impl::SplitPinchedVerts (src/edge_op.cpp:718-822).
//
// Iterates every halfedge fan; if a vertex has multiple non-contiguous
// fans rooted at it, the vertex is "pinched" and gets duplicated so
// each fan owns its own vertex copy. C++ has two branches (parallel
// when nbEdges > 1e4, serial otherwise); both produce equivalent
// topology with different fan-discovery orderings. We mirror BOTH
// branches.
func (mi *MutableImpl) SplitPinchedVerts() {
	nbEdges := mi.HalfedgeCount()
	if nbEdges == 0 {
		return
	}
	if nbEdges > 10000 {
		mi.splitPinchedVertsParallel()
		return
	}
	mi.splitPinchedVertsSerial()
}

// splitPinchedVertsSerial is the C++ #else / non-MANIFOLD_PAR branch
// (src/edge_op.cpp:798-820): linear walk over halfedges, single
// shared vertProcessed + halfedgeProcessed masks.
func (mi *MutableImpl) splitPinchedVertsSerial() {
	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	props := append([]int32(nil), mi.HalfedgeProps()...)
	srcVerts := append([]geom.Vec3(nil), mi.Verts()...)
	numEdge := len(starts)
	numVert := len(srcVerts)
	vertProcessed := make([]bool, numVert)
	halfedgeProcessed := make([]bool, numEdge)
	var newVerts []geom.Vec3
	for i := 0; i < numEdge; i++ {
		if halfedgeProcessed[i] {
			continue
		}
		v := int(starts[i])
		if v == -1 {
			continue
		}
		if vertProcessed[v] {
			newIdx := int32(numVert + len(newVerts))
			newVerts = append(newVerts, srcVerts[v])
			forVert(i, pairs, func(current int) {
				halfedgeProcessed[current] = true
				starts[current] = newIdx
				pair := int(pairs[current])
				starts[nextHalfedge(pair)] = newIdx
			})
			continue
		}
		vertProcessed[v] = true
		forVert(i, pairs, func(current int) {
			halfedgeProcessed[current] = true
		})
	}
	if len(newVerts) > 0 {
		mi.h.ResizeVerts(numVert + len(newVerts))
		copy(mi.Verts()[numVert:], newVerts)
	}
	mi.h.SetHalfedgesRaw(starts, props, pairs)
}

// splitPinchedVertsParallel is the C++ MANIFOLD_PAR branch
// (src/edge_op.cpp:722-795).
//
// Algorithm:
//  1. Initialize largestEdge[v] = MAX_INT64 for every vert (parallel).
//  2. Fan out across nbEdges; each worker holds a thread-local
//     visited[] mask. For each unvisited halfedge i:
//     - walk the ForVert fan, marking all visited locally;
//     - find the smallest halfedge index in the fan;
//     - atomically try CAS(largestEdge[vert], MAX, smallest). If
//     it succeeds, this fan is the first one rooted at this vert.
//     If it fails AND the existing value != smallest, this vert
//     has multiple fans → pinched. Push (smallest, existing) onto
//     a per-worker pinched list.
//  3. After parallel: gather worker pinched lists, sort + dedupe.
//  4. Serial walk over `pinched`: for each halfedge index, if its
//     startVert hasn't been processed yet, mark processed; otherwise
//     duplicate the vert and re-fan.
//
// Mirrors C++ exactly — tbb::combinable becomes per-goroutine local
// arrays, std::atomic compare_exchange_strong becomes
// atomic.CompareAndSwapInt64 + a reload on failure (since Go's CAS
// doesn't return the old value, we read it back to check whether
// the failure was due to a smallest-collision or a different fan).
func (mi *MutableImpl) splitPinchedVertsParallel() {
	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	props := append([]int32(nil), mi.HalfedgeProps()...)
	srcVerts := append([]geom.Vec3(nil), mi.Verts()...)
	nbEdges := len(starts)
	numVert := len(srcVerts)

	// Step 1: largestEdge[v] = MAX_INT64, in parallel.
	const sentinel int64 = math.MaxInt64
	largestEdge := make([]int64, numVert)
	parallel.ForEachN(parallel.Par, numVert, func(i int) {
		atomic.StoreInt64(&largestEdge[i], sentinel)
	})

	// Step 2: per-worker fan-out with thread-local visited masks +
	// thread-local pinched lists.
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > nbEdges {
		nWorkers = nbEdges
	}
	chunk := (nbEdges + nWorkers - 1) / nWorkers
	var pinchedMu sync.Mutex
	var pinched []int
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		s := w * chunk
		e := s + chunk
		if e > nbEdges {
			e = nbEdges
		}
		if s >= e {
			continue
		}
		wg.Add(1)
		go func(lo, hi int) {
			defer wg.Done()
			local := make([]bool, nbEdges)
			var pinchedLocal []int
			for i := lo; i < hi; i++ {
				if local[i] {
					continue
				}
				local[i] = true
				vert := int(starts[i])
				if vert == -1 {
					continue
				}
				smallest := int64(i)
				// ForVert walk — mirrors the C++ ForVert lambda body.
				current := i
				for {
					current = nextHalfedge(int(pairs[current]))
					local[current] = true
					if int64(current) < smallest {
						smallest = int64(current)
					}
					if current == i {
						break
					}
				}
				// Atomic compare-exchange. C++ uses
				//   compare_exchange_strong(got=MAX, smallest)
				// which atomically swaps if MAX and reloads `got`
				// on failure. Go's CAS doesn't return old; we
				// reload after a failed CAS to read the winning
				// value (same data — both writers wrote with
				// release semantics, our atomic Load has acquire).
				if !atomic.CompareAndSwapInt64(&largestEdge[vert], sentinel, smallest) {
					got := atomic.LoadInt64(&largestEdge[vert])
					if got != smallest {
						pinchedLocal = append(pinchedLocal, int(smallest), int(got))
					}
				}
			}
			if len(pinchedLocal) > 0 {
				pinchedMu.Lock()
				pinched = append(pinched, pinchedLocal...)
				pinchedMu.Unlock()
			}
		}(s, e)
	}
	wg.Wait()

	// Step 3: sort + dedupe pinched. C++ uses manifold::stable_sort
	// (src/edge_op.cpp:779) — match the primitive (default threshold 1e4).
	parallel.StableSort(parallel.AutoPolicy(len(pinched), 10000), pinched, func(a, b int) bool { return a < b })
	if len(pinched) > 1 {
		w := 1
		for i := 1; i < len(pinched); i++ {
			if pinched[i] != pinched[i-1] {
				pinched[w] = pinched[i]
				w++
			}
		}
		pinched = pinched[:w]
	}

	// Step 4: serial walk over pinched halfedges. For each, if its
	// startVert has been processed, duplicate the vert and re-fan.
	processedVerts := make(map[int]bool, len(pinched))
	var newVerts []geom.Vec3
	for _, i := range pinched {
		startVert := int(starts[i])
		if !processedVerts[startVert] {
			processedVerts[startVert] = true
			continue
		}
		newIdx := int32(numVert + len(newVerts))
		newVerts = append(newVerts, srcVerts[startVert])
		forVert(i, pairs, func(current int) {
			starts[current] = newIdx
			pair := int(pairs[current])
			starts[nextHalfedge(pair)] = newIdx
		})
	}
	if len(newVerts) > 0 {
		mi.h.ResizeVerts(numVert + len(newVerts))
		copy(mi.Verts()[numVert:], newVerts)
	}
	mi.h.SetHalfedgesRaw(starts, props, pairs)
}
