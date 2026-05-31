// Subdivision port (src/subdivision.cpp). This file holds the topology helpers
// (GetNeighbor / GetHalfedges / GetIndices / FillRetainedVerts + IsMarkedInsideQuad)
// and the Subdivide driver; the Partition triangulation cache lives in
// subdivide_partition.go and the Refine smoothing (InterpTri) in impl_refine.go.
// SubdivideN / Refine* are now fully native — no bridge.
package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// baryIndices is the Go port of Impl::BaryIndices (src/impl.h:52): the tri and the
// 0-3 corner indices (start4/end4) such that GetHalfedges(tri)[start4] points back
// to a given halfedge and end4 points to the next. Start4 < 0 marks a quad interior.
type baryIndices struct {
	Tri, Start4, End4 int
}

// isMarkedInsideQuad is the Go port of Impl::IsMarkedInsideQuad (smoothing.cpp:339):
// a halfedge is the interior diagonal of a quad iff its tangent's w is negative.
func (mi *MutableImpl) isMarkedInsideQuad(halfedge int) bool {
	return len(mi.s.halfedgeTangent) > 0 && mi.s.halfedgeTangent[halfedge].W < 0
}

// getNeighbor is the Go port of Impl::GetNeighbor (subdivision.cpp:389): the tri
// side index (0-2) joined to the other half of a quad, or -1 if this tri is not
// part of a quad (-2 if malformed: more than one inside-quad edge).
func (mi *MutableImpl) getNeighbor(tri int) int {
	neighbor := -1
	for i := 0; i < 3; i++ {
		if mi.isMarkedInsideQuad(3*tri + i) {
			if neighbor == -1 {
				neighbor = i
			} else {
				neighbor = -2
			}
		}
	}
	return neighbor
}

// getHalfedges is the Go port of Impl::GetHalfedges (subdivision.cpp:405): the
// three halfedges of a triangle with [3] = -1, OR the four halfedges of a quad
// (when this tri is the lower-indexed half), OR all -1s when this is the upper
// half of a quad.
func (mi *MutableImpl) getHalfedges(tri int) [4]int {
	halfedges := [4]int{-1, -1, -1, -1}
	for i := 0; i < 3; i++ {
		halfedges[i] = 3*tri + i
	}
	neighbor := mi.getNeighbor(tri)
	if neighbor >= 0 { // quad
		pair := mi.s.halfedge.Pair(3*tri + neighbor)
		if pair/3 < tri {
			return [4]int{-1, -1, -1, -1} // only process lower tri index
		}
		// The order matters to keep small quads split the way they started, else
		// it can create a 4-manifold edge.
		halfedges[2] = mesh.NextHalfedge(halfedges[neighbor])
		halfedges[3] = mesh.NextHalfedge(halfedges[2])
		halfedges[0] = mesh.NextHalfedge(pair)
		halfedges[1] = mesh.NextHalfedge(halfedges[0])
	}
	return halfedges
}

// getIndices is the Go port of Impl::GetIndices (subdivision.cpp:432): the
// BaryIndices for a halfedge, handling both triangles and quads; {-1,-1,-1} when
// the halfedge is the interior of a quad.
func (mi *MutableImpl) getIndices(halfedge int) baryIndices {
	tri := halfedge / 3
	idx := halfedge % 3
	neighbor := mi.getNeighbor(tri)
	if idx == neighbor {
		return baryIndices{-1, -1, -1}
	}
	if neighbor < 0 { // tri
		return baryIndices{tri, idx, geom.Next3(idx)}
	}
	// quad
	pair := mi.s.halfedge.Pair(3*tri + neighbor)
	if pair/3 < tri {
		tri = pair / 3
		if geom.Next3(neighbor) == idx {
			idx = 0
		} else {
			idx = 1
		}
	} else {
		if geom.Next3(neighbor) == idx {
			idx = 2
		} else {
			idx = 3
		}
	}
	return baryIndices{tri, idx, (idx + 1) % 4}
}

// fillRetainedVerts is the Go port of Impl::FillRetainedVerts (subdivision.cpp:460):
// seed each retained (original) vert's Barycentric from whichever incident
// triangle is visited last (order-independent for the result). Quad interiors are
// skipped.
func (mi *MutableImpl) fillRetainedVerts(vertBary []mesh.Barycentric) {
	numTri := mi.s.halfedge.Size() / 3
	for tri := 0; tri < numTri; tri++ {
		for i := 0; i < 3; i++ {
			indices := mi.getIndices(3*tri + i)
			if indices.Start4 < 0 {
				continue // skip quad interiors
			}
			var uvw geom.Vec4
			setVec4Comp(&uvw, indices.Start4, 1)
			vertBary[mi.s.halfedge.Start(3*tri+i)] = mesh.Barycentric{Tri: indices.Tri, UVW: uvw}
		}
	}
}

// setVec4Comp sets component i (0-3 -> X/Y/Z/W) of v. Mirrors C++ vec4 operator[].
func setVec4Comp(v *geom.Vec4, i int, val float64) {
	switch i {
	case 0:
		v.X = val
	case 1:
		v.Y = val
	case 2:
		v.Z = val
	case 3:
		v.W = val
	}
}

// getVec4Comp reads component i (0-3 -> X/Y/Z/W) of v.
func getVec4Comp(v geom.Vec4, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	case 2:
		return v.Z
	default:
		return v.W
	}
}

// flattenTriVerts flattens [][3]int triangle vert indices into the flat int32
// array CreateHalfedges expects (3 per triangle).
func flattenTriVerts(tris [][3]int) []int32 {
	out := make([]int32, 3*len(tris))
	for t, tv := range tris {
		out[3*t] = int32(tv[0])
		out[3*t+1] = int32(tv[1])
		out[3*t+2] = int32(tv[2])
	}
	return out
}

// Subdivide is the Go port of Manifold::Impl::Subdivide (src/subdivision.cpp:480):
// split each edge into edgeDivisions(...) pieces and sub-triangulate every face via
// the cached Partition triangulations. It does NOT run the finalize tail (that runs
// after the new verts are repositioned, e.g. by Refine), but it does rebuild the
// halfedge structure (CreateHalfedges). Returns the per-output-vert Barycentric
// coordinates so a caller (Refine) can place the new verts on the smooth surface.
func (mi *MutableImpl) Subdivide(edgeDivisions func(geom.Vec3, geom.Vec4, geom.Vec4) int, keepInterior bool) []mesh.Barycentric {
	edges := mesh.CreateTmpEdges(&mi.s.halfedge)
	numVert := mi.NumVert()
	numEdge := len(edges)
	numTri := mi.NumTri()
	policy := parallel.AutoPolicy(numEdge, 10000)

	half2Edge := make([]int, 2*numEdge)
	parallel.ForEachN(policy, numEdge, func(edge int) {
		idx := edges[edge].HalfedgeIdx
		half2Edge[idx] = edge
		half2Edge[mi.s.halfedge.Pair(idx)] = edge
	})

	faceHalfedges := make([][4]int, numTri)
	parallel.ForEachN(policy, numTri, func(tri int) {
		faceHalfedges[tri] = mi.getHalfedges(tri)
	})

	tangents := mi.s.halfedgeTangent
	edgeAdded := make([]int, numEdge)
	parallel.ForEachN(policy, numEdge, func(i int) {
		hIdx := edges[i].HalfedgeIdx
		if mi.isMarkedInsideQuad(hIdx) {
			edgeAdded[i] = 0
			return
		}
		vec := mi.s.vertPos[edges[i].First].Sub(mi.s.vertPos[edges[i].Second])
		var tangent0, tangent1 geom.Vec4
		if len(tangents) > 0 {
			tangent0 = tangents[hIdx]
			tangent1 = tangents[mi.s.halfedge.Pair(hIdx)]
		}
		edgeAdded[i] = edgeDivisions(vec, tangent0, tangent1)
	})

	if keepInterior {
		// Add extra divisions to short sides of strip-like triangles so the
		// triangulation has some thickness (subdivision.cpp:520-561).
		tmp := make([]int, numEdge)
		parallel.ForEachN(policy, numEdge, func(i int) {
			tmp[i] = edgeAdded[i]
			hIdx := edges[i].HalfedgeIdx
			if mi.isMarkedInsideQuad(hIdx) {
				return
			}
			thisAdded := tmp[i]
			added := func(hIdx int) int {
				longest := 0
				total := 0
				for k := 0; k < 3; k++ {
					a := edgeAdded[half2Edge[hIdx]]
					if a > longest {
						longest = a
					}
					total += a
					hIdx = mesh.NextHalfedge(hIdx)
					if mi.isMarkedInsideQuad(hIdx) {
						longest = 0
						total = 1
						break
					}
				}
				minExtra := int(float64(longest)*0.2 + 1)
				extra := 2*longest + minExtra - total
				if longest == 0 {
					return 0
				}
				if extra > 0 {
					return (extra * (longest - thisAdded)) / longest
				}
				return 0
			}
			a0 := added(hIdx)
			a1 := added(mi.s.halfedge.Pair(hIdx))
			if a1 > a0 {
				tmp[i] += a1
			} else {
				tmp[i] += a0
			}
		})
		edgeAdded = tmp
	}

	edgeOffset := make([]int, numEdge)
	parallel.ExclusiveScan(parallel.AutoPolicy(numEdge, 100000), edgeAdded, edgeOffset, numVert)

	vertBarySize := numVert
	if numEdge > 0 {
		vertBarySize = edgeOffset[numEdge-1] + edgeAdded[numEdge-1]
	}
	vertBary := make([]mesh.Barycentric, vertBarySize)
	totalEdgeAdded := vertBarySize - numVert
	mi.fillRetainedVerts(vertBary)
	parallel.ForEachN(policy, numEdge, func(i int) {
		n := edgeAdded[i]
		offset := edgeOffset[i]
		indices := mi.getIndices(edges[i].HalfedgeIdx)
		if indices.Tri < 0 {
			return // inside quad
		}
		frac := 1.0 / float64(n+1)
		for j := 0; j < n; j++ {
			endVal := float64(j+1) * frac
			var uvw geom.Vec4
			setVec4Comp(&uvw, indices.End4, endVal)
			setVec4Comp(&uvw, indices.Start4, 1-endVal)
			vertBary[offset+j].UVW = uvw
			vertBary[offset+j].Tri = indices.Tri
		}
	})

	subTris := make([]partition, numTri)
	parallel.ForEachN(policy, numTri, func(tri int) {
		halfedges := faceHalfedges[tri]
		var divisions [4]int
		for _, i := range []int{0, 1, 2, 3} {
			if halfedges[i] >= 0 {
				divisions[i] = edgeAdded[half2Edge[halfedges[i]]] + 1
			}
		}
		subTris[tri] = getPartition(divisions)
	})

	scanPolicy := parallel.AutoPolicy(numTri, 100000)
	triOffset := make([]int, numTri)
	sizes := make([]int, numTri)
	for t := range subTris {
		sizes[t] = len(subTris[t].triVert)
	}
	parallel.ExclusiveScan(scanPolicy, sizes, triOffset, 0)

	interiorOffset := make([]int, numTri)
	numInts := make([]int, numTri)
	for t := range subTris {
		numInts[t] = subTris[t].numInterior()
	}
	parallel.ExclusiveScan(scanPolicy, numInts, interiorOffset, len(vertBary))

	numOutTri := 0
	if numTri > 0 {
		numOutTri = triOffset[numTri-1] + len(subTris[numTri-1].triVert)
	}
	triVerts := make([][3]int, numOutTri)
	newVertBarySize := len(vertBary)
	if numTri > 0 {
		newVertBarySize = interiorOffset[numTri-1] + subTris[numTri-1].numInterior()
	}
	if newVertBarySize > len(vertBary) {
		vertBary = append(vertBary, make([]mesh.Barycentric, newVertBarySize-len(vertBary))...)
	}
	triRef := make([]mesh.TriRef, len(triVerts))
	faceNormalOut := make([]geom.Vec3, len(triVerts))

	oldTriRef := mi.s.meshRelation.TriRef
	oldFaceNormal := mi.s.faceNormal
	parallel.ForEachN(policy, numTri, func(tri int) {
		halfedges := faceHalfedges[tri]
		if halfedges[0] < 0 {
			return
		}
		var tri3 [4]int
		var edgeOffsets [4]int
		var edgeFwd [4]bool
		for _, i := range []int{0, 1, 2, 3} {
			if halfedges[i] < 0 {
				tri3[i] = -1
				continue
			}
			tri3[i] = mi.s.halfedge.Start(halfedges[i])
			edgeOffsets[i] = edgeOffset[half2Edge[halfedges[i]]]
			edgeFwd[i] = mi.s.halfedge.IsForward(halfedges[i])
		}
		newTris := subTris[tri].reindex(tri3, edgeOffsets, edgeFwd, interiorOffset[tri])
		copy(triVerts[triOffset[tri]:], newTris)
		for k := range newTris {
			triRef[triOffset[tri]+k] = oldTriRef[tri]
			faceNormalOut[triOffset[tri]+k] = oldFaceNormal[tri]
		}

		idx := subTris[tri].idx
		vIdx := idx
		if !(halfedges[3] >= 0 || idx[1] == geom.Next3(idx[0])) {
			vIdx = [4]int{idx[2], idx[0], idx[1], idx[3]}
		}
		var rIdx [4]int
		for _, i := range []int{0, 1, 2, 3} {
			rIdx[vIdx[i]] = i
		}
		subBary := subTris[tri].vertBary
		io := subTris[tri].interiorOffset()
		for k := io; k < len(subBary); k++ {
			bary := subBary[k]
			vertBary[interiorOffset[tri]+(k-io)] = mesh.Barycentric{
				Tri: tri,
				UVW: geom.Vec4{
					X: getVec4Comp(bary, rIdx[0]), Y: getVec4Comp(bary, rIdx[1]),
					Z: getVec4Comp(bary, rIdx[2]), W: getVec4Comp(bary, rIdx[3]),
				},
			}
		}
	})
	mi.s.meshRelation.TriRef = triRef
	mi.s.faceNormal = faceNormalOut

	newVertPos := make([]geom.Vec3, len(vertBary))
	parallel.ForEachN(policy, len(vertBary), func(vert int) {
		bary := vertBary[vert]
		halfedges := faceHalfedges[bary.Tri]
		u := bary.UVW
		if halfedges[3] < 0 {
			var triPos [3]geom.Vec3
			for _, i := range []int{0, 1, 2} {
				triPos[i] = mi.s.vertPos[mi.s.halfedge.Start(halfedges[i])]
			}
			newVertPos[vert] = triPos[0].Scale(u.X).Add(triPos[1].Scale(u.Y)).Add(triPos[2].Scale(u.Z))
		} else {
			var quadPos [4]geom.Vec3
			for _, i := range []int{0, 1, 2, 3} {
				quadPos[i] = mi.s.vertPos[mi.s.halfedge.Start(halfedges[i])]
			}
			newVertPos[vert] = quadPos[0].Scale(u.X).Add(quadPos[1].Scale(u.Y)).
				Add(quadPos[2].Scale(u.Z)).Add(quadPos[3].Scale(u.W))
		}
	})
	mi.s.vertPos = newVertPos

	if mi.s.numProp > 0 {
		numProp := mi.s.numProp
		oldProps := mi.s.properties
		numPropVert := len(oldProps) / numProp
		addedVerts := mi.NumVert() - numVert
		propOffset := numPropVert - numVert
		// Duplicate prop verts along all new edges (CompactProps removes dups later).
		prop := make([]float64, numProp*(numPropVert+addedVerts+totalEdgeAdded))
		copy(prop, oldProps)

		// interior + forward-edge prop verts
		parallel.ForEachN(policy, addedVerts, func(i int) {
			vert := numPropVert + i
			bary := vertBary[numVert+i]
			halfedges := faceHalfedges[bary.Tri]
			u := bary.UVW
			for p := 0; p < numProp; p++ {
				if halfedges[3] < 0 {
					var triProp [3]float64
					for _, k := range []int{0, 1, 2} {
						triProp[k] = oldProps[mi.s.halfedge.Prop(3*bary.Tri+k)*numProp+p]
					}
					prop[vert*numProp+p] = triProp[0]*u.X + triProp[1]*u.Y + triProp[2]*u.Z
				} else {
					var quadProp [4]float64
					for _, k := range []int{0, 1, 2, 3} {
						quadProp[k] = oldProps[mi.s.halfedge.Prop(halfedges[k])*numProp+p]
					}
					prop[vert*numProp+p] = quadProp[0]*u.X + quadProp[1]*u.Y + quadProp[2]*u.Z + quadProp[3]*u.W
				}
			}
		})

		// backward-edge prop verts (some are unreferenced dups)
		parallel.ForEachN(policy, numEdge, func(i int) {
			n := edgeAdded[i]
			offset := edgeOffset[i] + propOffset + addedVerts
			frac := 1.0 / float64(n+1)
			halfedgeIdx := mi.s.halfedge.Pair(edges[i].HalfedgeIdx)
			prop0 := mi.s.halfedge.Prop(halfedgeIdx)
			prop1 := mi.s.halfedge.Prop(mesh.NextHalfedge(halfedgeIdx))
			for j := 0; j < n; j++ {
				for p := 0; p < numProp; p++ {
					prop[(offset+j)*numProp+p] = lerpScalar(
						oldProps[prop0*numProp+p], oldProps[prop1*numProp+p], float64(j+1)*frac)
				}
			}
		})

		triProp := make([][3]int, len(triVerts))
		parallel.ForEachN(policy, numTri, func(tri int) {
			halfedges := faceHalfedges[tri]
			if halfedges[0] < 0 {
				return
			}
			var tri3 [4]int
			var edgeOffsets [4]int
			edgeFwd := [4]bool{true, true, true, true}
			for _, i := range []int{0, 1, 2, 3} {
				if halfedges[i] < 0 {
					tri3[i] = -1
					continue
				}
				tri3[i] = mi.s.halfedge.Prop(halfedges[i])
				edgeOffsets[i] = edgeOffset[half2Edge[halfedges[i]]]
				if !mi.s.halfedge.IsForward(halfedges[i]) {
					pair := mi.s.halfedge.Pair(halfedges[i])
					if mi.s.halfedge.Prop(pair) != mi.s.halfedge.Prop(mesh.NextHalfedge(halfedges[i])) ||
						mi.s.halfedge.Prop(mesh.NextHalfedge(pair)) != mi.s.halfedge.Prop(halfedges[i]) {
						edgeOffsets[i] += addedVerts
					} else {
						edgeFwd[i] = false
					}
				}
			}
			var eo [4]int
			for k := range edgeOffsets {
				eo[k] = edgeOffsets[k] + propOffset
			}
			newTris := subTris[tri].reindex(tri3, eo, edgeFwd, interiorOffset[tri]+propOffset)
			copy(triProp[triOffset[tri]:], newTris)
		})

		mi.s.properties = prop
		mi.CreateHalfedges(flattenTriVerts(triProp), flattenTriVerts(triVerts))
	} else {
		mi.CreateHalfedges(flattenTriVerts(triVerts), nil)
	}

	return vertBary
}
