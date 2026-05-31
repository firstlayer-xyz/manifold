// Subdivision port (src/subdivision.cpp). This file currently holds the topology
// helpers that Subdivide builds on (GetNeighbor / GetHalfedges / GetIndices /
// FillRetainedVerts + IsMarkedInsideQuad); the Partition triangulation cache and
// the Subdivide driver itself land in follow-on increments. Until then, the
// SubdivideN / Refine* methods still run through the C++ bridge (runBridgeAlgo).
package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
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
