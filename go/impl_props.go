package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// matchesTriNormals is the Go port of C++ Manifold::Impl::MatchesTriNormals
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
func implMatchesTriNormals(impl *bridge.Impl) bool {
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	pairs := impl.HalfedgePairs()
	normals := impl.FaceNormals()
	numTri := len(starts) / 3
	if numTri == 0 || len(normals) != numTri {
		return true
	}
	s := impl.Scalars()
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
		for i := 0; i < 3; i++ {
			p := verts[starts[3*face+i]]
			v[i] = proj.MulVec3(p)
			d := p.Dot(normal)
			if math.IsInf(d, 0) || math.IsNaN(d) {
				// Non-finite projection: C++ short-circuits the lambda to true.
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

// numDegenerateTris is the Go port of C++ Manifold::Impl::NumDegenerateTris.
// For each interior triangle (Pair >= 0), if its axis-aligned 2D projection
// is colinear within tolerance_/2, count it. Boundary halfedges (Pair < 0)
// are also counted as degenerate, mirroring the C++ `return true` early
// return inside count_if.
//
// Note the C++ early return:
//
//	if (halfedge_.size() == 0 || faceNormal_.size() != NumTri())
//	    return true;   // <-- yes, returns size_t-cast-of-bool; mesh empty/no normals → 1
//
// matches that quirk: when there are no triangles or no face normals,
// return 1 (the C++ implicit bool-to-int conversion of `true`).
func implNumDegenerateTris(impl *bridge.Impl) int {
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	pairs := impl.HalfedgePairs()
	normals := impl.FaceNormals()
	numTri := len(starts) / 3
	if numTri == 0 || len(normals) != numTri {
		return 1
	}
	s := impl.Scalars()
	tol := s.Tolerance / 2
	count := 0
	for face := 0; face < numTri; face++ {
		if pairs[3*face] < 0 {
			count++
			continue
		}
		proj := geom.GetAxisAlignedProjection(normals[face])
		var v [3]geom.Vec2
		for i := 0; i < 3; i++ {
			v[i] = proj.MulVec3(verts[starts[3*face+i]])
		}
		if geom.CCW(v[0], v[1], v[2], tol) == 0 {
			count++
		}
	}
	return count
}
