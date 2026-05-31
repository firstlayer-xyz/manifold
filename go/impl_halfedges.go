package manifold

import (
	"runtime"
	"sort"
	"sync"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// CreateHalfedges is the Go port of the single-arg form of C++
// Manifold::Impl::CreateHalfedges (src/impl.cpp:381). triVerts is
// flat: 3 int32 indices per triangle. Builds halfedge_.start_,
// .propVert_, .paired_ and writes them via SetHalfedgesRaw.
//
// Faithful to C++ across both branches:
//   - vertCount < 1<<18 → sort-based pairing (PrepHalfedges with
//     setEdge sort-key, then stable_sort ids by edge[]).
//   - vertCount >= 1<<18 → bucket-based pairing (PrepHalfedges with
//     atomic offset counters, exclusive_scan, scatter into entries,
//     local sort each bucket).
//
// The duplicate-triangle detection body (C++ `body` lambda) detects
// when two triangles share the same (start, end, next-end-vert) tuple:
// they get marked `removed` and the `ids` array is shuffled so
// remaining edges still pair up.
func (mi *MutableImpl) CreateHalfedges(triProp []int32, triVert []int32) {
	numHalfedge := len(triProp)
	if numHalfedge%3 != 0 {
		panic("CreateHalfedges: triProp length must be a multiple of 3")
	}
	if len(triVert) != 0 && len(triVert) != numHalfedge {
		panic("CreateHalfedges: triVert must be empty or the same length as triProp")
	}
	numTri := numHalfedge / 3
	numEdge := numHalfedge / 2
	policy := parallel.AutoPolicy(numTri, 100000)

	// PrepHalfedges (src/impl.cpp:352-372): build per-halfedge
	// (startVert, endVert, propVert) records. startVert/endVert come from
	// triVert (or triProp when triVert is empty — the single-arg form,
	// where startVert == propVert); propVert ALWAYS comes from triProp.
	// The edge key / bucket offset is computed from startVert/endVert.
	type heRec struct {
		startVert, endVert, propVert int32
	}
	he := make([]heRec, numHalfedge)

	// C++ uses vertPos_.size() (src/impl.cpp:389) — the impl's vert
	// count, not a value derived from triVerts. This feeds both the
	// `< (1<<18)` branch selection and the bucket-branch offset sizing.
	vertCount := mi.NumVert()
	ids := make([]int32, numHalfedge)

	if vertCount < (1 << 18) {
		// Sort-based branch (src/impl.cpp:393-413). Build edge keys
		// then stable_sort ids by edge keys.
		key := make([]uint64, numHalfedge)
		parallel.ForEachN(policy, numTri, func(tri int) {
			for i := 0; i < 3; i++ {
				j := i + 1
				if j == 3 {
					j = 0
				}
				e := 3*tri + i
				p0 := triProp[3*tri+i]
				v0, v1 := p0, triProp[3*tri+j]
				if len(triVert) > 0 {
					v0 = triVert[3*tri+i]
					v1 = triVert[3*tri+j]
				}
				if v0 == v1 {
					panic("CreateHalfedges: topological degeneracy (v0 == v1)")
				}
				he[e] = heRec{startVert: v0, endVert: v1, propVert: p0}
				var fwd uint64
				minV, maxV := v0, v1
				if v0 < v1 {
					fwd = 1
				} else {
					minV, maxV = v1, v0
				}
				key[e] = (fwd << 63) | (uint64(uint32(minV)) << 32) | uint64(uint32(maxV))
			}
		})
		parallel.Sequence(policy, ids)
		// stable_sort default threshold 1e4 (parallel.h:1140).
		parallel.StableSort(parallel.AutoPolicy(numHalfedge, 10000), ids, func(a, b int32) bool {
			return key[a] < key[b]
		})
	} else {
		// Bucket-based branch (src/impl.cpp:414-456). Counter per
		// (min(v0,v1) + (v0>v1 ? 0 : vertCount)) bucket, exclusive
		// scan to compute starts, scatter halfedges into entries[]
		// at offset positions, then local sort each bucket.
		offsets := make([]int32, vertCount*2)
		parallel.ForEachN(policy, numTri, func(tri int) {
			for i := 0; i < 3; i++ {
				j := i + 1
				if j == 3 {
					j = 0
				}
				e := 3*tri + i
				p0 := triProp[3*tri+i]
				v0, v1 := p0, triProp[3*tri+j]
				if len(triVert) > 0 {
					v0 = triVert[3*tri+i]
					v1 = triVert[3*tri+j]
				}
				if v0 == v1 {
					panic("CreateHalfedges: topological degeneracy (v0 == v1)")
				}
				he[e] = heRec{startVert: v0, endVert: v1, propVert: p0}
				var off int32
				if v0 <= v1 {
					off = int32(vertCount)
				}
				minV := v0
				if v1 < minV {
					minV = v1
				}
				atomic.AddInt32(&offsets[minV+off], 1)
			}
		})
		// Exclusive scan: offsets[i] becomes sum(offsets[0..i-1]).
		// Mirrors src/impl.cpp:435 `exclusive_scan(offsets, offsets, offsets)`.
		exclScan := make([]int32, len(offsets))
		parallel.ExclusiveScan(policy, offsets, exclScan, 0)
		offsets = exclScan

		// Scatter halfedges into entries[] at their bucket offset.
		type entry struct {
			largeVert int32
			tri       int32
			edgeIndex int32
		}
		entries := make([]entry, numHalfedge)
		parallel.ForEachN(policy, numTri, func(tri int) {
			for i := 0; i < 3; i++ {
				e := 3*tri + i
				v0 := he[e].startVert
				v1 := he[e].endVert
				var off int32
				if v0 <= v1 {
					off = int32(vertCount)
				}
				minV, maxV := v0, v1
				if v1 < v0 {
					minV, maxV = v1, v0
				}
				idx := atomic.AddInt32(&offsets[minV+off], 1) - 1
				entries[idx] = entry{largeVert: maxV, tri: int32(tri), edgeIndex: int32(e)}
			}
		})

		// Local sort each bucket. C++ does sequential sort per bucket
		// via for_each_n(policy, ..., offsets.size(), ...). We mirror
		// with parallel.ForEachN over the offset count.
		parallel.ForEachN(policy, len(offsets), func(v int) {
			start := 0
			if v > 0 {
				start = int(offsets[v-1])
			}
			end := int(offsets[v])
			if start >= end {
				return
			}
			// Fill ids[start:end] = [start..end) then sort by entries.
			for i := start; i < end; i++ {
				ids[i] = int32(i)
			}
			sub := ids[start:end]
			sort.Slice(sub, func(a, b int) bool {
				ea := entries[sub[a]]
				eb := entries[sub[b]]
				if ea.largeVert != eb.largeVert {
					return ea.largeVert < eb.largeVert
				}
				return ea.tri < eb.tri
			})
			// Translate from entries-index back to edgeIndex.
			for i := start; i < end; i++ {
				ids[i] = entries[ids[i]].edgeIndex
			}
		})
	}

	// After PrepHalfedges + sort: ids[0..numEdge) are backward
	// halfedges (v0 >= v1) and ids[numEdge..2*numEdge) are forward
	// halfedges (v0 < v1). Each backward edge at ids[i] pairs with
	// the forward edge at ids[i+numEdge] — provided there are no
	// duplicates. The body lambda below detects and removes
	// duplicates, reshuffling ids[] to keep remaining edges paired.
	removed := make([]bool, numHalfedge)

	body := func(i, consecutiveStart, segmentEnd int) int {
		pair0 := int(ids[i])
		h0 := he[pair0]
		k := consecutiveStart + numEdge
		for {
			pair1 := int(ids[k])
			h1 := he[pair1]
			if h0.startVert != h1.endVert || h0.endVert != h1.startVert {
				break
			}
			if !removed[pair1] && he[nextHalfedge(pair0)].endVert == he[nextHalfedge(pair1)].endVert {
				removed[pair0] = true
				removed[pair1] = true
				if i+numEdge != k {
					// Reorder ids[] so remaining edges still pair up,
					// preserving relative order. Mirrors C++
					// src/impl.cpp:481-498.
					dir := 1
					if i+numEdge >= k {
						dir = -1
					}
					a := k
					b := k + dir
					inRange := func() bool {
						if dir > 0 {
							return a >= i+numEdge
						}
						return a <= i+numEdge
					}
					isRemoved := func(x int) bool { return removed[ids[x]] }
					for {
						for {
							a -= dir
							if !inRange() || !isRemoved(a) {
								break
							}
						}
						if !inRange() {
							break
						}
						for {
							b -= dir
							if !isRemoved(b) || b == k {
								break
							}
						}
						ids[b] = ids[a]
					}
					ids[i+numEdge] = int32(pair1)
				}
				break
			}
			k++
			if k >= segmentEnd+numEdge {
				break
			}
		}
		if i+1 == segmentEnd {
			return consecutiveStart
		}
		nextH := he[ids[i+1]]
		if nextH.startVert == h0.startVert && nextH.endVert == h0.endVert {
			return consecutiveStart
		}
		return i + 1
	}

	// Outer body loop, mirroring the C++ parallel variant: partition
	// ids into ranges keeping duplicate-edge segments contiguous, then
	// parallel_for over ranges. The serial variant is one linear pass;
	// both produce the same output.
	if numEdge > 10000 {
		// Build ranges, each ~ numEdge/(2*maxConcurrency) sized,
		// extended to ensure duplicate edges stay in the same range.
		concurrency := runtime.GOMAXPROCS(0)
		increment := numEdge / concurrency / 2
		if increment < 1024 {
			increment = 1024
		}
		if increment > numEdge {
			increment = numEdge
		}
		duplicated := func(a, b int) bool {
			h0 := he[ids[a]]
			h1 := he[ids[b]]
			return h0.startVert == h1.startVert && h0.endVert == h1.endVert
		}
		type rng struct{ start, end int }
		var ranges []rng
		end := 0
		for end < numEdge {
			start := end
			end += increment
			if end > numEdge {
				end = numEdge
			}
			for end < numEdge && duplicated(end-1, end) {
				end++
			}
			ranges = append(ranges, rng{start, end})
		}
		var wg sync.WaitGroup
		for _, r := range ranges {
			r := r
			wg.Add(1)
			go func() {
				defer wg.Done()
				consecutiveStart := r.start
				for i := r.start; i < r.end; i++ {
					consecutiveStart = body(i, consecutiveStart, r.end)
				}
			}()
		}
		wg.Wait()
	} else {
		consecutiveStart := 0
		for i := 0; i < numEdge; i++ {
			consecutiveStart = body(i, consecutiveStart, numEdge)
		}
	}

	// Write halfedge_ arrays. C++ runs this as for_each_n over numEdge.
	starts := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	paireds := make([]int32, numHalfedge)
	parallel.ForEachN(policy, numEdge, func(i int) {
		pair0 := int(ids[i])
		pair1 := int(ids[i+numEdge])
		if !removed[pair0] {
			starts[pair0] = he[pair0].startVert
			props[pair0] = he[pair0].propVert
			paireds[pair0] = int32(pair1)
			starts[pair1] = he[pair1].startVert
			props[pair1] = he[pair1].propVert
			paireds[pair1] = int32(pair0)
		} else {
			starts[pair0] = -1
			props[pair0] = 0
			paireds[pair0] = -1
			starts[pair1] = -1
			props[pair1] = 0
			paireds[pair1] = -1
		}
	})
	mi.SetHalfedgesRaw(starts, props, paireds)
}
