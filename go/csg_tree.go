// CSG-tree port (src/csg_tree.cpp). In the Go port, Manifold.Transform is eager
// (it materializes immediately rather than deferring through CsgNode::Transform),
// so Go never builds nested CsgOpNode trees and every Manifold leaf carries an
// identity transform. That collapses the C++ lazy-tree machinery (CsgNode /
// CsgOpNode / the recursive ToLeafNode DFS, the deferred-transform branches) to
// just the n-ary boolean evaluator behind the static BatchBoolean: Compose +
// BatchBoolean(heap) + BatchUnion. This file holds that evaluator, ported from
// csg_tree.cpp and operating on the manifold-package Impl facade.
//
// The deferred-transform branches of CsgLeafNode::Compose (src/csg_tree.cpp:
// 340-372, 395-397) are intentionally NOT ported: they only run for leaves with
// a non-identity transform_, which Go's eager model never produces. If lazy
// transform fusion is ever added, they must be ported too.
package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// compose is the Go port of CsgLeafNode::Compose (src/csg_tree.cpp:209): the
// efficient union of a set of pairwise-disjoint meshes. It concatenates the
// leaves' vertices / halfedges / faces / properties (offsetting indices and
// meshIDs per leaf), then runs RemoveDegenerates -> SortGeometry ->
// IncrementMeshIDs to produce one combined leaf.
//
// Inputs are const Impls with an implied identity transform (the Go eager model
// — see the file header). A non-NoError input short-circuits to an errored
// empty Impl, mirroring the C++ early return.
func compose(leaves []*Impl) *MutableImpl {
	epsilon := -1.0
	tolerance := -1.0
	bbox := geom.EmptyBox()
	numVert, numEdge, numTri, numPropVert := 0, 0, 0, 0
	numPropOut := 0
	n := len(leaves)
	vertIndices := make([]int, n)
	edgeIndices := make([]int, n)
	triIndices := make([]int, n)
	propVertIndices := make([]int, n)

	for i, node := range leaves {
		sc := node.Scalars()
		if Error(sc.Status) != NoError {
			mi := newImpl()
			mi.MakeEmpty(sc.Status)
			return mi
		}
		// Identity transform: newScale == oldScale, so the C++ ratio max(1,
		// newScale/oldScale) is 1 and nodeEpsilon starts at epsilon_.
		minB, maxB := node.BBox()
		nodeBox := geom.Box{Min: minB, Max: maxB}
		nodeEpsilon := sc.Epsilon
		nodeEpsilon = math.Max(nodeEpsilon, kPrecision*nodeBox.Scale())
		if math.IsInf(nodeEpsilon, 0) || math.IsNaN(nodeEpsilon) {
			nodeEpsilon = -1
		}
		epsilon = math.Max(epsilon, nodeEpsilon)
		tolerance = math.Max(tolerance, sc.Tolerance)
		bbox = bbox.Union(nodeBox)

		vertIndices[i] = numVert
		edgeIndices[i] = numEdge * 2
		triIndices[i] = numTri
		propVertIndices[i] = numPropVert
		numVert += node.NumVert()
		numEdge += node.HalfedgeCount() / 2
		numTri += node.NumTri()
		numProp := sc.NumProp
		if numProp > numPropOut {
			numPropOut = numProp
		}
		if numProp == 0 {
			numPropVert++
		} else {
			numPropVert += len(node.Properties()) / numProp
		}
	}

	numHalfedge := 2 * numEdge
	vertPos := make([]geom.Vec3, numVert)
	vertNormal := make([]geom.Vec3, numVert)
	faceNormal := make([]geom.Vec3, numTri)
	starts := make([]int32, numHalfedge)
	paireds := make([]int32, numHalfedge)
	props := make([]int32, numHalfedge)
	tangents := make([]float64, 4*numHalfedge) // C++ unconditionally resizes to 2*numEdge
	triMeshID := make([]int32, numTri)
	triOriginalID := make([]int32, numTri)
	triFaceID := make([]int32, numTri)
	triCoplanarID := make([]int32, numTri)
	var properties []float64
	if numPropOut > 0 {
		properties = make([]float64, numPropOut*numPropVert)
	}

	// meshIDCounter_ snapshot (fetch_add 0): the per-instance meshID offset, so
	// repeated copies of the same mesh in `leaves` get distinct meshIDs.
	meshIDCounterSnapshot := int32(reserveIDs(0))

	for i, node := range leaves {
		nextVert := int32(vertIndices[i])
		nextEdge := int32(edgeIndices[i])
		nextProp := int32(propVertIndices[i])
		numProp := node.Scalars().NumProp
		hasProp := numProp > 0

		nStarts := node.HalfedgeStarts()
		nPairs := node.HalfedgePairs()
		nProps := node.HalfedgeProps()
		for edge := range nStarts {
			newEdge := edgeIndices[i] + edge
			starts[newEdge] = nStarts[edge] + nextVert
			paireds[newEdge] = nPairs[edge] + nextEdge
			if hasProp {
				props[newEdge] = nProps[edge] + nextProp
			} else {
				props[newEdge] = nextProp
			}
		}

		// halfedgeTangent copy into the leaf's halfedge range (flat 4-per).
		nTangents := node.HalfedgeTangents()
		copy(tangents[4*edgeIndices[i]:], nTangents)

		// Properties: strided copy per prop slot (combined stride numPropOut).
		if hasProp {
			oldProp := node.Properties()
			nPV := len(oldProp) / numProp
			for p := 0; p < numProp; p++ {
				for v := 0; v < nPV; v++ {
					properties[numPropOut*(propVertIndices[i]+v)+p] = oldProp[numProp*v+p]
				}
			}
		}

		// Identity transform: straight copies (the on-the-fly transform branch
		// is unreachable in the eager model — see file header).
		copy(vertPos[vertIndices[i]:], node.Verts())
		copy(vertNormal[vertIndices[i]:], node.VertNormals())
		copy(faceNormal[triIndices[i]:], node.FaceNormals())

		// triRef with per-instance meshID offset.
		offset := int32(i) * meshIDCounterSnapshot
		refs := node.TriRefs()
		for t, r := range refs {
			triMeshID[triIndices[i]+t] = int32(r.MeshID) + offset
			triOriginalID[triIndices[i]+t] = int32(r.OriginalID)
			triFaceID[triIndices[i]+t] = int32(r.FaceID)
			triCoplanarID[triIndices[i]+t] = int32(r.CoplanarID)
		}
	}

	combined := newImpl()
	combined.SetEpsilonValue(epsilon)
	combined.SetToleranceValue(tolerance)
	combined.SetBBox(bbox.Min, bbox.Max)
	combined.ResizeVerts(numVert)
	copy(combined.Verts(), vertPos)
	combined.ResizeVertNormals(numVert)
	copy(combined.VertNormals(), vertNormal)
	combined.ResizeFaceNormals(numTri)
	copy(combined.FaceNormals(), faceNormal)
	combined.SetHalfedgesRaw(starts, props, paireds)
	combined.SetHalfedgeTangents(tangents)
	if numPropOut > 0 {
		combined.SetNumProp(numPropOut)
		combined.SetProperties(properties)
	}
	combined.SetTriRefs(triMeshID, triOriginalID, triFaceID, triCoplanarID)

	// meshIDtransform: copy each leaf's entries with the per-instance key offset
	// (identity transform leaves rel.transform unchanged).
	combined.ClearMeshIDTransforms()
	for i, node := range leaves {
		offset := i * int(meshIDCounterSnapshot)
		for _, r := range node.MeshIDTransforms() {
			combined.AddMeshIDTransform(int(r.MeshID)+offset, int(r.OriginalID),
				r.Transform, r.BackSide, r.HasNormals)
		}
	}

	combined.RemoveDegenerates(0)
	combined.SortGeometry()
	combined.IncrementMeshIDs()
	return combined
}

// composeManifolds runs compose over a set of leaf Manifolds and seals the
// result. Mirrors the C++ CsgLeafNode::Compose call sites in BatchUnion.
func composeManifolds(ms []*Manifold) *Manifold {
	impls := make([]*Impl, len(ms))
	for i, m := range ms {
		impls[i] = getImpl(m)
	}
	defer func() {
		for _, im := range impls {
			im.Delete()
		}
	}()
	return compose(impls).ToManifold()
}

// simpleBoolean is the Go port of SimpleBoolean (src/csg_tree.cpp:157): a single
// two-operand Boolean3 evaluation. In the Go port that is exactly the native
// Manifold.Boolean (nativeBoolean3), so it just delegates.
func simpleBoolean(a, b *Manifold, op OpType) *Manifold { return a.Boolean(b, op) }

// leafBBox returns a leaf's bounding box. In the eager model the leaf transform
// is identity, so this is the Impl's bBox_ (CsgLeafNode::GetBoundingBox).
func leafBBox(m *Manifold) geom.Box {
	i := getImpl(m)
	defer i.Delete()
	mn, mx := i.BBox()
	return geom.Box{Min: mn, Max: mx}
}

// batchBoolean is the Go port of BatchBoolean (src/csg_tree.cpp:413): the
// commutative n-ary union/intersection (Add/Intersect only) that repeatedly
// combines meshes, smaller results re-entering the pool. The C++ keeps a max-heap
// keyed by (NumVert, insertion serial) and pops the two greatest per step, four
// pairs per round. Because the serial makes that ordering TOTAL, the pop sequence
// is fixed by the comparator alone — a linear max-select reproduces the exact
// pairing order (and the 4-pairs-per-round batching) without depending on heap
// internals. SEQ path (serials assigned in pop order).
func batchBoolean(op OpType, results []*Manifold) *Manifold {
	if len(results) == 0 {
		return emptyManifold(NoError)
	}
	if len(results) == 1 {
		return results[0]
	}
	if len(results) == 2 {
		return simpleBoolean(results[0], results[1], op)
	}
	type heapNode struct {
		node    *Manifold
		numVert int
		serial  uint64
	}
	heap := make([]heapNode, len(results))
	for i, m := range results {
		heap[i] = heapNode{node: m, numVert: m.NumVert(), serial: uint64(i)}
	}
	nextSerial := uint64(len(heap))
	// popMax removes and returns the element with the greatest (numVert, serial)
	// — the std::pop_heap target under MeshCompare.
	popMax := func() heapNode {
		best := 0
		for i := 1; i < len(heap); i++ {
			if heap[i].numVert != heap[best].numVert {
				if heap[i].numVert > heap[best].numVert {
					best = i
				}
				continue
			}
			if heap[i].serial > heap[best].serial {
				best = i
			}
		}
		e := heap[best]
		heap = append(heap[:best], heap[best+1:]...)
		return e
	}
	for len(heap) > 1 {
		var tmp []heapNode
		for i := 0; i < 4 && len(heap) > 1; i++ {
			a := popMax()
			b := popMax()
			result := simpleBoolean(a.node, b.node, op)
			tmp = append(tmp, heapNode{node: result, numVert: result.NumVert(), serial: nextSerial})
			nextSerial++
		}
		heap = append(heap, tmp...)
	}
	return heap[0].node
}

// batchUnion is the Go port of BatchUnion (src/csg_tree.cpp:486): union a set of
// nodes by composing as many pairwise-disjoint subsets as possible (cheap
// concatenation) before falling back to batchBoolean for the overlapping
// remainder. Chunks at kMaxUnionSize to bound the O(n^2) disjointness check.
// Operates on a private copy of children so the caller's slice is not mutated.
func batchUnion(childrenIn []*Manifold) *Manifold {
	const kMaxUnionSize = 1000
	children := append([]*Manifold(nil), childrenIn...)
	for len(children) > 1 {
		start := 0
		if len(children) > kMaxUnionSize {
			start = len(children) - kMaxUnionSize
		}
		boxes := make([]geom.Box, 0, len(children)-start)
		for i := start; i < len(children); i++ {
			boxes = append(boxes, leafBBox(children[i]))
		}
		// Partition into pairwise-disjoint sets: greedy first-fit into the first
		// set none of whose members overlap boxes[i], else a new set.
		var disjointSets [][]int
		for i := 0; i < len(boxes); i++ {
			placed := false
			for s := range disjointSets {
				overlaps := false
				for _, j := range disjointSets[s] {
					if boxes[i].DoesOverlap(boxes[j]) {
						overlaps = true
						break
					}
				}
				if !overlaps {
					disjointSets[s] = append(disjointSets[s], i)
					placed = true
					break
				}
			}
			if !placed {
				disjointSets = append(disjointSets, []int{i})
			}
		}
		// Compose each disjoint set (singletons pass through unchanged).
		var impls []*Manifold
		for _, set := range disjointSets {
			if len(set) == 1 {
				impls = append(impls, children[start+set[0]])
			} else {
				tmp := make([]*Manifold, len(set))
				for k, j := range set {
					tmp[k] = children[start+j]
				}
				impls = append(impls, composeManifolds(tmp))
			}
		}
		children = children[:start]
		children = append(children, batchBoolean(OpAdd, impls))
		// Move the freshly-combined (likely complex) child to the front, since we
		// process from the back.
		children[0], children[len(children)-1] = children[len(children)-1], children[0]
	}
	return children[0]
}
