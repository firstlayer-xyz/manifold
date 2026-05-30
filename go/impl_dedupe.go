package manifold

import (
	"runtime"
	"sync"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// dedupeState holds Go-side copies of all the impl arrays that
// DedupeEdges grows or mutates. We keep everything in Go for the
// duration of the algorithm (which may run multiple outer-loop
// passes) and sync back to the bridge once at the end. Mirrors the
// fields that the C++ DedupeEdge body touches:
// vertPos_, vertNormal_, halfedge_ (start/paired/propVert),
// meshRelation_.triRef, faceNormal_.
type dedupeState struct {
	verts       []geom.Vec3
	vertNormals []geom.Vec3
	starts      []int32
	pairs       []int32
	props       []int32
	triRefs     []bridge.TriRef
	faceNormals []geom.Vec3
	properties  []float64 // properties_ — property values (numProp per propVert)
	numProp     int       // numProp_ — gates prop updates in collapse/swap
	epsilon     float64   // epsilon_ — collapse tolerance floor
	tolerance   float64   // tolerance_ — edge-swap degeneracy threshold
}

func newDedupeState(mi *MutableImpl) *dedupeState {
	return &dedupeState{
		verts:       append([]geom.Vec3(nil), mi.Verts()...),
		vertNormals: append([]geom.Vec3(nil), mi.VertNormals()...),
		starts:      append([]int32(nil), mi.HalfedgeStarts()...),
		pairs:       append([]int32(nil), mi.HalfedgePairs()...),
		props:       append([]int32(nil), mi.HalfedgeProps()...),
		triRefs:     append([]bridge.TriRef(nil), mi.TriRefs()...),
		faceNormals: append([]geom.Vec3(nil), mi.FaceNormals()...),
		properties:  append([]float64(nil), mi.Properties()...),
		numProp:     mi.NumProp(),
		epsilon:     mi.Epsilon(),
		tolerance:   mi.Tolerance(),
	}
}

// Halfedge accessors mirror the inline C++ Halfedges methods.

func (s *dedupeState) numHalfedge() int { return len(s.starts) }

func (s *dedupeState) start(e int) int32 { return s.starts[e] }
func (s *dedupeState) end(e int) int32   { return s.starts[nextHalfedge(e)] }
func (s *dedupeState) pair(e int) int32  { return s.pairs[e] }
func (s *dedupeState) prop(e int) int32  { return s.props[e] }

func (s *dedupeState) setStart(e int, v int32) { s.starts[e] = v }
func (s *dedupeState) setEnd(e int, v int32)   { s.starts[nextHalfedge(e)] = v }
func (s *dedupeState) setPair(e int, p int32)  { s.pairs[e] = p }
func (s *dedupeState) setProp(e int, p int32)  { s.props[e] = p }

// pushHalfedge grows the halfedge arrays by one, returning the new
// halfedge index. Mirrors C++ halfedge_.push_back(start, pair, prop)
// — note pair is set later via pairUp; we pass -1 here matching C++.
func (s *dedupeState) pushHalfedge(start, pair, prop int32) int {
	idx := len(s.starts)
	s.starts = append(s.starts, start)
	s.pairs = append(s.pairs, pair)
	s.props = append(s.props, prop)
	return idx
}

// pushVert grows vertPos_ (and vertNormal_ if present), returning
// the new vert index.
func (s *dedupeState) pushVert(p geom.Vec3, normalSrc int) int {
	idx := len(s.verts)
	s.verts = append(s.verts, p)
	if len(s.vertNormals) > 0 {
		s.vertNormals = append(s.vertNormals, s.vertNormals[normalSrc])
	}
	return idx
}

// pushTri grows triRef and faceNormal arrays by one each, copying
// from the supplied source face. Mirrors C++ push_back on both arrays
// guarded by their non-empty checks.
func (s *dedupeState) pushTri(srcFace int) {
	if len(s.triRefs) > 0 {
		s.triRefs = append(s.triRefs, s.triRefs[srcFace])
	}
	if len(s.faceNormals) > 0 {
		s.faceNormals = append(s.faceNormals, s.faceNormals[srcFace])
	}
}

// pairUp is the Go port of C++ Manifold::Impl::PairUp (src/edge_op.cpp).
// Symmetric write: pairs[e0] = e1, pairs[e1] = e0.
func (s *dedupeState) pairUp(e0, e1 int) {
	s.pairs[e0] = int32(e1)
	s.pairs[e1] = int32(e0)
}

// updateVert is the Go port of C++ Manifold::Impl::UpdateVert
// (src/edge_op.cpp). Traverses CW around the shared endVert from
// startEdge to endEdge (endEdge.endVert must equal startEdge.endVert),
// rewriting each visited halfedge's End and Start to point at the
// new vert. The C++ DEBUG_ASSERT against an infinite loop is
// preserved as a panic — hitting it means the input topology
// invariants are violated.
func (s *dedupeState) updateVert(vert int32, startEdge, endEdge int) {
	current := startEdge
	for current != endEdge {
		s.setEnd(current, vert)
		current = nextHalfedge(current)
		s.setStart(current, vert)
		current = int(s.pair(current))
		if current == startEdge {
			panic("dedupeState.updateVert: infinite loop")
		}
	}
}

// forVertS is the dedupeState equivalent of forVert (impl_halfedge_walk.go).
// We can't reuse forVert directly because it takes a fixed pairs
// slice, but s.pairs may grow during the walk.
func (s *dedupeState) forVertS(start int, fn func(edge int)) {
	current := start
	for {
		current = nextHalfedge(int(s.pair(current)))
		fn(current)
		if current == start {
			return
		}
	}
}

// dedupeEdge is the Go port of C++ Manifold::Impl::DedupeEdge
// (src/edge_op.cpp:325). Splits the duplicate edge by either
//   - adding two new triangles (the "single topological unit"
//     branch, when the orbit around endVert finds startVert again),
//   - or duplicating the vert and re-fanning (the "separate
//     topological unit" branch when the orbit completes).
//
// The trailing pinched-startVert check mirrors the C++: after the
// previous split, the OTHER end may also be pinched and needs a
// second duplication.
//
// Mirrors the C++ algorithm exactly, including the (apparent)
// detail that the second pinch-split uses endVert's position as
// the new vert's pos — matches src/edge_op.cpp:401 verbatim.
func (s *dedupeState) dedupeEdge(edge int) {
	nextE := nextHalfedge(edge)
	startVert := s.start(edge)
	endVert := s.start(nextE)
	endProp := s.prop(nextE)
	current := int(s.pair(nextE))

	// Orbit endVert looking for startVert (the "single topological
	// unit" trigger).
	found := false
	for current != edge {
		if s.start(current) == startVert {
			// Single unit — duplicate endVert and add two new triangles.
			newVert := int32(s.pushVert(s.verts[endVert], int(endVert)))
			current = int(s.pair(nextHalfedge(current)))
			opposite := int(s.pair(nextE))

			s.updateVert(newVert, current, opposite)

			// First new triangle — bridges between `current` and `opposite`.
			newHalfedge := len(s.starts)
			oldFace := current / 3
			outsideVert := s.start(current)
			s.pushHalfedge(endVert, -1, endProp)
			s.pushHalfedge(newVert, -1, endProp)
			s.pushHalfedge(outsideVert, -1, s.prop(current))
			s.pairUp(newHalfedge+2, int(s.pair(current)))
			s.pairUp(newHalfedge+1, current)
			s.pushTri(oldFace)

			newHalfedge += 3
			oldFace = opposite / 3
			outsideVert = s.start(opposite)
			s.pushHalfedge(newVert, -1, endProp)
			s.pushHalfedge(endVert, -1, endProp)
			s.pushHalfedge(outsideVert, -1, s.prop(opposite))
			s.pairUp(newHalfedge+2, int(s.pair(opposite)))
			s.pairUp(newHalfedge+1, opposite)
			s.pairUp(newHalfedge, newHalfedge-3)
			s.pushTri(oldFace)

			found = true
			break
		}
		current = int(s.pair(nextHalfedge(current)))
	}

	if !found {
		// Separate topological unit — simpler split, no new faces.
		newVert := int32(s.pushVert(s.verts[endVert], int(endVert)))
		s.forVertS(nextHalfedge(current), func(e int) {
			s.setStart(e, newVert)
			s.setEnd(int(s.pair(e)), newVert)
		})
	}

	// Orbit startVert and check if it's pinched too.
	pair := int(s.pair(edge))
	current = int(s.pair(nextHalfedge(pair)))
	for current != pair {
		if s.start(current) == endVert {
			return // connected to endVert — not pinched
		}
		current = int(s.pair(nextHalfedge(current)))
	}
	// Pinched startVert from the previous split: duplicate the
	// vert (note the C++ uses endVert's POSITION, not startVert's
	// — preserved verbatim).
	newVert := int32(s.pushVert(s.verts[endVert], int(endVert)))
	s.forVertS(nextHalfedge(current), func(e int) {
		s.setStart(e, newVert)
		s.setEnd(int(s.pair(e)), newVert)
	})
}

// localLoop is the Go port of the C++ `localLoop` lambda in
// DedupeEdges (src/edge_op.cpp:829-902). Walks halfedges [start,end)
// looking for duplicate-edge cycles, marking visited slots in
// `local` and appending duplicate halfedge indices to `results`.
//
// The two-tier collection structure (Vec linear-search for fans of
// size ≤ 32, unordered_map promotion for larger fans) is preserved
// verbatim — at high vertex degrees the linear search would be
// quadratic, so C++ swaps to a hashmap once the fan exceeds 32
// distinct end-verts. We mirror with a Go []struct + a Go map[int32]
// promotion.
func (s *dedupeState) localLoop(start, end int, local []bool, results *[]int) {
	// Reused-allocation scratch space, matching the C++
	// `endVerts.clear(false)` keep-capacity pattern.
	endVerts := make([]struct{ endV, current int32 }, 0, 32)
	var endVertSet map[int32]int32
	for i := start; i < end; i++ {
		if local[i] {
			continue
		}
		startVert := s.start(i)
		endVert := s.end(i)
		if startVert == -1 || endVert == -1 {
			continue
		}
		endVerts = endVerts[:0]
		if endVertSet != nil {
			for k := range endVertSet {
				delete(endVertSet, k)
			}
		}

		// First iteration: populate (endV, minHalfedge) entries.
		s.forVertS(i, func(current int) {
			local[current] = true
			sV := s.start(current)
			eV := s.end(current)
			if sV == -1 || eV == -1 {
				return
			}
			if endVertSet == nil {
				// Linear-search path (fan size ≤ 32).
				for k := range endVerts {
					if endVerts[k].endV == eV {
						if int32(current) < endVerts[k].current {
							endVerts[k].current = int32(current)
						}
						return
					}
				}
				endVerts = append(endVerts, struct{ endV, current int32 }{eV, int32(current)})
				if len(endVerts) > 32 {
					// Promote to map — matches C++ src/edge_op.cpp:872-875.
					endVertSet = make(map[int32]int32, len(endVerts)*2)
					for _, p := range endVerts {
						endVertSet[p.endV] = p.current
					}
					endVerts = endVerts[:0]
				}
			} else {
				existing, ok := endVertSet[eV]
				if !ok {
					endVertSet[eV] = int32(current)
				} else if int32(current) < existing {
					endVertSet[eV] = int32(current)
				}
			}
		})

		// Second iteration: report halfedges that aren't the min
		// for their (start, end) tuple as duplicates.
		s.forVertS(i, func(current int) {
			sV := s.start(current)
			eV := s.end(current)
			if sV == -1 || eV == -1 {
				return
			}
			if endVertSet == nil {
				for k := range endVerts {
					if endVerts[k].endV == eV {
						if int(endVerts[k].current) != current {
							*results = append(*results, current)
						}
						return
					}
				}
			} else {
				min := endVertSet[eV]
				if int(min) != current {
					*results = append(*results, current)
				}
			}
		})
	}
}

// findDuplicatesSerial is the C++ non-MANIFOLD_PAR branch
// (src/edge_op.cpp:926-929): one full-range localLoop with a
// single shared visited mask.
func (s *dedupeState) findDuplicatesSerial() []int {
	local := make([]bool, s.numHalfedge())
	var results []int
	s.localLoop(0, s.numHalfedge(), local, &results)
	return results
}

// findDuplicatesParallel is the C++ MANIFOLD_PAR branch
// (src/edge_op.cpp:904-923): fan out across nbEdges with
// per-worker visited masks (tbb::combinable<vector<bool>>) and
// per-worker results lists merged at the end. After the parallel
// phase, results are stable_sort'd + unique'd to match C++.
func (s *dedupeState) findDuplicatesParallel() []int {
	nbEdges := s.numHalfedge()
	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > nbEdges {
		nWorkers = nbEdges
	}
	chunk := (nbEdges + nWorkers - 1) / nWorkers
	var mu sync.Mutex
	var results []int
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		lo := w * chunk
		hi := lo + chunk
		if hi > nbEdges {
			hi = nbEdges
		}
		if lo >= hi {
			continue
		}
		wg.Add(1)
		go func(s2 *dedupeState, lo, hi int) {
			defer wg.Done()
			// Thread-local visited mask — matches the
			// tbb::combinable<vector<bool>> pattern.
			local := make([]bool, nbEdges)
			var localResults []int
			s2.localLoop(lo, hi, local, &localResults)
			if len(localResults) > 0 {
				mu.Lock()
				results = append(results, localResults...)
				mu.Unlock()
			}
		}(s, lo, hi)
	}
	wg.Wait()
	// C++ manifold::stable_sort + unique (src/edge_op.cpp:920). Match
	// the primitive (default stable_sort threshold 1e4), then dedupe.
	parallel.StableSort(parallel.AutoPolicy(len(results), 10000), results, func(a, b int) bool { return a < b })
	if len(results) > 1 {
		w := 1
		for i := 1; i < len(results); i++ {
			if results[i] != results[i-1] {
				results[w] = results[i]
				w++
			}
		}
		results = results[:w]
	}
	return results
}

// commit syncs the dedupeState's arrays back to the underlying
// bridge MutableImpl. Called once at the end of DedupeEdges.
func (s *dedupeState) commit(mi *MutableImpl) {
	// Resize vert buffers if grown.
	if len(s.verts) != mi.NumVert() {
		mi.ResizeVerts(len(s.verts))
	}
	copy(mi.Verts(), s.verts)
	if len(s.vertNormals) > 0 {
		mi.ResizeVertNormals(len(s.vertNormals))
		copy(mi.VertNormals(), s.vertNormals)
	}
	mi.SetHalfedgesRaw(s.starts, s.props, s.pairs)
	if len(s.triRefs) > 0 {
		meshIDs := make([]int32, len(s.triRefs))
		originalIDs := make([]int32, len(s.triRefs))
		faceIDs := make([]int32, len(s.triRefs))
		coplanarIDs := make([]int32, len(s.triRefs))
		for i, r := range s.triRefs {
			meshIDs[i] = r.MeshID
			originalIDs[i] = r.OriginalID
			faceIDs[i] = r.FaceID
			coplanarIDs[i] = r.CoplanarID
		}
		mi.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
	}
	if len(s.faceNormals) > 0 {
		mi.ResizeFaceNormals(len(s.faceNormals))
		copy(mi.FaceNormals(), s.faceNormals)
	}
	// properties_ may have grown (RecursiveEdgeSwap interpolates new
	// property verts); sync the whole buffer back.
	if len(s.properties) > 0 {
		mi.SetProperties(s.properties)
	}
}

// DedupeEdges is the Go port of C++ Manifold::Impl::DedupeEdges
// (src/edge_op.cpp:823). Iteratively detects duplicate halfedges
// (multiple halfedges sharing the same (start, end) vertex pair
// within a single startVert fan) and splits them via dedupeEdge,
// looping until no duplicates remain.
//
// Matches C++ exactly: parallel detection branch when nbEdges > 1e4
// (tbb::combinable per-worker visited masks + mutex-protected
// results merge + stable_sort + unique), serial detection branch
// otherwise. The two-tier fan-collection (linear-search Vec for
// ≤32 entries, unordered_map promotion above) is preserved inside
// localLoop.
func (mi *MutableImpl) DedupeEdges() {
	s := newDedupeState(mi)
	for {
		var dups []int
		if s.numHalfedge() > 10000 {
			dups = s.findDuplicatesParallel()
		} else {
			dups = s.findDuplicatesSerial()
		}
		if len(dups) == 0 {
			break
		}
		for _, e := range dups {
			s.dedupeEdge(e)
		}
	}
	s.commit(mi)
}

// CleanupTopology is the Go port of C++ Manifold::Impl::CleanupTopology
// (src/edge_op.cpp:108). Splits pinched verts and dedupes parallel
// edges — the two correctness fixes that must precede edge
// collapse / swap operations on a freshly-built mesh.
//
// The C++ DEBUG_ASSERT(IsManifold()) precondition is omitted here
// (no debug mode) but the assumption is the same: callers must
// pass a valid 2-manifold-ish mesh.
func (mi *MutableImpl) CleanupTopology() {
	if mi.HalfedgeCount() == 0 {
		return
	}
	mi.SplitPinchedVerts()
	mi.DedupeEdges()
}
