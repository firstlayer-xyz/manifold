package manifold

// CompactProps is the Go port of C++ Manifold::Impl::CompactProps
// (src/sort.cpp). Removes unreferenced property verts and reindexes
// the halfedge_.propVert_ slots.
//
// Early return when numProp_ == 0 (nothing to compact). For the
// common case in our test suite — primitives with no extra
// properties — this is a no-op, matching C++.
func (mi *MutableImpl) CompactProps() {
	numProp := mi.NumProp()
	if numProp == 0 {
		return
	}
	props := mi.Properties()
	halfedgeProps := append([]int32(nil), mi.HalfedgeProps()...)
	numVerts := len(props) / numProp

	// keep[propVert] = 1 if any halfedge references it.
	keep := make([]int, numVerts)
	for _, p := range halfedgeProps {
		keep[p] = 1
	}

	// propOld2New: inclusive_scan of keep — propOld2New[i+1] gives the
	// new index for old vert i (when kept).
	propOld2New := make([]int32, numVerts+1)
	var sum int32
	for i := 0; i < numVerts; i++ {
		sum += int32(keep[i])
		propOld2New[i+1] = sum
	}
	numVertsNew := int(propOld2New[numVerts])

	// Gather kept properties into a new buffer.
	newProps := make([]float64, numProp*numVertsNew)
	for oldIdx := 0; oldIdx < numVerts; oldIdx++ {
		if keep[oldIdx] == 0 {
			continue
		}
		newIdx := int(propOld2New[oldIdx])
		for p := 0; p < numProp; p++ {
			newProps[newIdx*numProp+p] = props[oldIdx*numProp+p]
		}
	}
	mi.h.SetProperties(newProps)

	// Reindex halfedge_.propVert_ via propOld2New. Read the other two
	// halfedge_ arrays unchanged and write them all back via the bulk
	// SetHalfedgesRaw mutator.
	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	for i, p := range halfedgeProps {
		halfedgeProps[i] = propOld2New[p]
	}
	mi.h.SetHalfedgesRaw(starts, halfedgeProps, pairs)
}
