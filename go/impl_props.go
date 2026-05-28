package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// MatchesTriNormals is the Go port of C++ Manifold::Impl::MatchesTriNormals
// (src/properties.cpp). For each triangle, the face is projected onto
// the 2D plane axis-aligned with its precomputed face normal; a
// CCW-positive 2D orientation (with the configured epsilon tolerance)
// must agree with the recorded normal direction. Boundary halfedges
// (Pair < 0) are accepted unconditionally, mirroring the C++ all_of.
//
// Both empty meshes and meshes without cached face normals trivially
// pass — matching C++:
//
//	if (halfedge_.size() == 0 || faceNormal_.size() != NumTri())
//	    return true;
func (i *Impl) MatchesTriNormals() bool {
	verts := i.Verts()
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	normals := i.FaceNormals()
	numTri := len(starts) / 3
	if numTri == 0 || len(normals) != numTri {
		return true
	}
	s := i.Scalars()
	tolerance := s.Tolerance
	epsilon := s.Epsilon
	for face := 0; face < numTri; face++ {
		if pairs[3*face] < 0 {
			continue
		}
		normal := normals[face]
		proj := geom.GetAxisAlignedProjection(normal)
		var v [3]geom.Vec2
		maxD := math.Inf(-1)
		minD := math.Inf(1)
		boundary := false
		for k := 0; k < 3; k++ {
			p := verts[starts[3*face+k]]
			v[k] = proj.MulVec3(p)
			d := p.Dot(normal)
			if math.IsInf(d, 0) || math.IsNaN(d) {
				boundary = true
				break
			}
			if d > maxD {
				maxD = d
			}
			if d < minD {
				minD = d
			}
		}
		if boundary {
			continue
		}
		if maxD-minD > 2*tolerance {
			return false
		}
		if geom.CCW(v[0], v[1], v[2], epsilon*2) < 0 {
			return false
		}
	}
	return true
}

// NumDegenerateTris is the Go port of C++
// Manifold::Impl::NumDegenerateTris. For each interior triangle
// (Pair >= 0), if its axis-aligned 2D projection is colinear within
// tolerance_/2, count it. Boundary halfedges (Pair < 0) are also
// counted as degenerate, mirroring the C++ `return true` early
// return inside count_if.
//
// Note the C++ early return:
//
//	if (halfedge_.size() == 0 || faceNormal_.size() != NumTri())
//	    return true;   // <-- yes, returns size_t-cast-of-bool
//
// matches that quirk: when there are no triangles or no face normals,
// return 1 (the C++ implicit bool-to-int conversion of `true`).
func (i *Impl) NumDegenerateTris() int {
	verts := i.Verts()
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	normals := i.FaceNormals()
	numTri := len(starts) / 3
	if numTri == 0 || len(normals) != numTri {
		return 1
	}
	s := i.Scalars()
	tol := s.Tolerance / 2
	count := 0
	for face := 0; face < numTri; face++ {
		if pairs[3*face] < 0 {
			count++
			continue
		}
		proj := geom.GetAxisAlignedProjection(normals[face])
		var v [3]geom.Vec2
		for k := 0; k < 3; k++ {
			v[k] = proj.MulVec3(verts[starts[3*face+k]])
		}
		if geom.CCW(v[0], v[1], v[2], tol) == 0 {
			count++
		}
	}
	return count
}
