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

	"github.com/firstlayer-xyz/manifold/go/bridge"
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
	meshIDCounterSnapshot := int32(bridge.ImplReserveIDs(0))

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
			triMeshID[triIndices[i]+t] = r.MeshID + offset
			triOriginalID[triIndices[i]+t] = r.OriginalID
			triFaceID[triIndices[i]+t] = r.FaceID
			triCoplanarID[triIndices[i]+t] = r.CoplanarID
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
