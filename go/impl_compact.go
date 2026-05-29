package manifold

import (
	"math"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/disjointsets"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// RemoveUnreferencedVerts is the Go port of C++
// Manifold::Impl::RemoveUnreferencedVerts (src/impl.cpp). For every
// halfedge with startVert >= 0, mark its startVert as kept; every
// other vertPos_ entry is set to NaN. Subsequent SortVerts maps NaN
// positions to NoMortonCode, sorting them to the tail and trimming
// them off — that is the C++ contract.
//
// The C++ uses policy = autoPolicy(numVert, 1e5) and two for_each_n
// passes; the first stores to keep[startVert] atomically because the
// same startVert may be visited by multiple halfedges concurrently.
// We mirror that with sync/atomic on a []int32 keep buffer.
func (mi *MutableImpl) RemoveUnreferencedVerts() {
	starts := mi.HalfedgeStarts()
	verts := mi.Verts()
	numVert := len(verts)
	if numVert == 0 {
		return
	}
	keep := make([]int32, numVert)
	policy := parallel.AutoPolicy(numVert, 100000)
	parallel.ForEachN(policy, len(starts), func(edge int) {
		startVert := starts[edge]
		if startVert >= 0 {
			atomic.StoreInt32(&keep[startVert], 1)
		}
	})
	nan := math.NaN()
	parallel.ForEachN(policy, numVert, func(v int) {
		if keep[v] == 0 {
			verts[v] = geom.Vec3{X: nan, Y: nan, Z: nan}
		}
	})
}

// DedupePropVerts is the Go port of C++
// Manifold::Impl::DedupePropVerts (src/impl.cpp). For every shared
// edge whose two halfedges belong to triangles with the same meshID,
// if the property vector at the edge's start vert is exactly equal
// across the two sides, the two property-vert indices are merged
// into one. Merging is done via union-find (DisjointSets) over the
// candidate-equality "edges"; each resulting component is collapsed
// to a single representative propVert index. The unreferenced
// propVerts are then removed by a subsequent CompactProps call.
//
// Early return when numProp_ == 0 — there is nothing to dedupe.
// The C++ uses policy = autoPolicy(halfedge_.size(), 1e4).
func (mi *MutableImpl) DedupePropVerts() {
	numProp := mi.NumProp()
	if numProp == 0 {
		return
	}
	starts := mi.HalfedgeStarts()
	pairs := mi.HalfedgePairs()
	halfedgeProps := append([]int32(nil), mi.HalfedgeProps()...)
	triRefs := mi.TriRefs()
	properties := mi.Properties()
	numHalfedge := len(starts)

	// vert2vert[edgeIdx] = (prop0, prop1) when the shared-edge property
	// vectors match — i.e. these two propVerts should be merged.
	vert2vert := make([][2]int32, numHalfedge)
	for i := range vert2vert {
		vert2vert[i] = [2]int32{-1, -1}
	}
	policy := parallel.AutoPolicy(numHalfedge, 10000)
	parallel.ForEachN(policy, numHalfedge, func(edgeIdx int) {
		pair := int(pairs[edgeIdx])
		if pair < 0 {
			return
		}
		edgeFace := edgeIdx / 3
		pairFace := pair / 3
		if triRefs[edgeFace].MeshID != triRefs[pairFace].MeshID {
			return
		}
		prop0 := halfedgeProps[edgeIdx]
		prop1 := halfedgeProps[nextHalfedge(pair)]
		for p := 0; p < numProp; p++ {
			if properties[numProp*int(prop0)+p] != properties[numProp*int(prop1)+p] {
				return
			}
		}
		vert2vert[edgeIdx] = [2]int32{prop0, prop1}
	})

	// GetLabels (impl.cpp): union-find over the merge-candidate edges,
	// then connected-components labeling over the propVerts.
	numPropVert := len(properties) / numProp
	uf := disjointsets.New(numPropVert)
	for _, e := range vert2vert {
		if e[0] == -1 || e[1] == -1 {
			continue
		}
		uf.Unite(int(e[0]), int(e[1]))
	}
	vertLabels := make([]int32, numPropVert)
	numLabels := uf.ConnectedComponents(vertLabels)

	// label2vert: walk every propVert in order, recording the last vert
	// seen for each label as the representative. Matches the C++ loop
	// `for v in [0, numPropVert) label2vert[vertLabels[v]] = v;` — a
	// serial fold whose result depends on order, not parallelizable.
	label2vert := make([]int32, numLabels)
	for v := 0; v < numPropVert; v++ {
		label2vert[vertLabels[v]] = int32(v)
	}

	// Rewrite halfedge_.propVert_ via the label-representative table.
	// C++ wraps this in a SetProp call per edge; the work per index is
	// trivial and independent, so the rewrite is parallel.
	parallel.ForEachN(policy, numHalfedge, func(edge int) {
		halfedgeProps[edge] = label2vert[vertLabels[halfedgeProps[edge]]]
	})
	startsCopy := append([]int32(nil), starts...)
	pairsCopy := append([]int32(nil), pairs...)
	mi.h.SetHalfedgesRaw(startsCopy, halfedgeProps, pairsCopy)
}

// CompactProps is the Go port of C++ Manifold::Impl::CompactProps
// (src/sort.cpp). Removes unreferenced property verts and reindexes
// the halfedge_.propVert_ slots.
//
// Three parallel passes (matching C++):
//  1. Mark every propVert referenced by some halfedge — keep[Prop(i)]
//     is written atomically because multiple halfedges share verts.
//  2. Inclusive scan of keep into propOld2New[1:] — gives the new
//     index for each kept vert (and the trim count at the tail).
//  3. Gather kept properties into the resized properties_ buffer,
//     and rewrite halfedge_.propVert_ via propOld2New.
//
// Early return when numProp_ == 0 (nothing to compact).
func (mi *MutableImpl) CompactProps() {
	numProp := mi.NumProp()
	if numProp == 0 {
		return
	}
	props := mi.Properties()
	halfedgeProps := append([]int32(nil), mi.HalfedgeProps()...)
	numVerts := len(props) / numProp
	numHalfedge := len(halfedgeProps)
	policy := parallel.AutoPolicy(numVerts, 100000)

	// Pass 1: mark referenced propVerts. Multiple halfedges may
	// reference the same propVert, so we use atomic stores — C++ uses
	// std::atomic<int>::store(1, relaxed).
	keep := make([]int32, numVerts)
	parallel.ForEachN(policy, numHalfedge, func(i int) {
		atomic.StoreInt32(&keep[halfedgeProps[i]], 1)
	})

	// Pass 2: inclusive scan into propOld2New[1:]; propOld2New[i+1]
	// gives the new index of old vert i when kept, and propOld2New[n]
	// is the new total count.
	propOld2New := make([]int32, numVerts+1)
	parallel.InclusiveScan(policy, keep, propOld2New[1:])
	numVertsNew := int(propOld2New[numVerts])

	// Pass 3a: gather kept properties into a new buffer.
	newProps := make([]float64, numProp*numVertsNew)
	parallel.ForEachN(policy, numVerts, func(oldIdx int) {
		if keep[oldIdx] == 0 {
			return
		}
		newIdx := int(propOld2New[oldIdx])
		for p := 0; p < numProp; p++ {
			newProps[newIdx*numProp+p] = props[oldIdx*numProp+p]
		}
	})
	mi.h.SetProperties(newProps)

	// Pass 3b: rewrite halfedge_.propVert_ via propOld2New.
	parallel.ForEachN(policy, numHalfedge, func(i int) {
		halfedgeProps[i] = propOld2New[halfedgeProps[i]]
	})
	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	mi.h.SetHalfedgesRaw(starts, halfedgeProps, pairs)
}
