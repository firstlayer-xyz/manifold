package manifold

import (
	"math"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
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
//
// C++ uses the no-policy `all_of` form which defaults to threshold
// 1e5 via autoPolicy. We mirror with parallel.AllOf and the same
// default threshold.
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
	return parallel.AllOf(parallel.AutoPolicy(numTri, 100000), numTri, func(face int) bool {
		if pairs[3*face] < 0 {
			return true
		}
		normal := normals[face]
		proj := geom.GetAxisAlignedProjection(normal)
		var v [3]geom.Vec2
		maxD := math.Inf(-1)
		minD := math.Inf(1)
		for k := 0; k < 3; k++ {
			p := verts[starts[3*face+k]]
			v[k] = proj.MulVec3(p)
			d := p.Dot(normal)
			if math.IsInf(d, 0) || math.IsNaN(d) {
				return true
			}
			if d > maxD {
				maxD = d
			}
			if d < minD {
				minD = d
			}
		}
		if maxD-minD > 2*tolerance {
			return false
		}
		return geom.CCW(v[0], v[1], v[2], epsilon*2) >= 0
	})
}

// IsSelfIntersecting is the Go port of C++
// Manifold::Impl::IsSelfIntersecting (src/properties.cpp). True iff
// any two non-adjacent triangles in the mesh have overlapping
// geometry, with an epsilon-tolerant proximity test on the four
// face-normal-perturbed configurations to avoid flagging
// epsilon-coincident adjacent geometry.
//
// Algorithm:
//  1. Build per-face boxes + Morton codes and feed them to the
//     Collider.
//  2. Query self-collisions over those boxes — for each (tri0,
//     tri1) overlap pair, run the per-pair check:
//     a. Skip if any pair of verts (one from each tri) is within
//        epsilon — that means the tris share a vertex region.
//     b. If DistanceTriangleTriangleSquared == 0 (geometrically
//        touching), try perturbing each tri by ±epsilon along the
//        OTHER tri's face normal. If any perturbation pulls them
//        apart (dist > 0), the original was a near-coincidence
//        (not a true intersection); skip.
//     c. Otherwise: real self-intersection; record true.
//  3. Return true iff any pair flagged.
//
// The Collider's parallel collision visit may invoke the recorder
// concurrently — we use sync/atomic.Bool for the result flag.
func (i *Impl) IsSelfIntersecting() bool {
	starts := i.HalfedgeStarts()
	verts := i.Verts()
	faceNormals := i.FaceNormals()
	if len(starts) == 0 || len(faceNormals)*3 != len(starts) {
		return false
	}
	scalars := i.Scalars()
	ep := 2 * scalars.Epsilon
	epsilonSq := ep * ep

	// Build Collider over per-face boxes (matches the persistent
	// C++ collider_ field). Mirrors what Slice does — see the
	// PORT_NOTES note about ephemerally rebuilt Colliders.
	faceBox, faceMorton, faceID := perFaceBoxMorton(i)
	if len(faceBox) == 0 {
		return false
	}
	policy := parallel.AutoPolicy(len(faceBox), 100000)
	order := make([]int32, len(faceBox))
	parallel.Sequence(policy, order)
	parallel.StableSort(parallel.AutoPolicy(len(faceBox), 10000), order, func(a, b int32) bool {
		return faceMorton[a] < faceMorton[b]
	})
	sortedBox := make([]geom.Box, len(faceBox))
	sortedCode := make([]uint32, len(faceBox))
	sortedFaceID := make([]int32, len(faceBox))
	parallel.ForEachN(policy, len(faceBox), func(newIdx int) {
		oldIdx := order[newIdx]
		sortedBox[newIdx] = faceBox[oldIdx]
		sortedCode[newIdx] = faceMorton[oldIdx]
		sortedFaceID[newIdx] = faceID[oldIdx]
	})
	c := collider.New(sortedBox, sortedCode)

	// Recorder: invoked once per (queryLeaf, otherLeaf) overlap.
	// selfCollision skips queryLeaf == otherLeaf. C++ does NOT
	// short-circuit on first hit — every pair gets checked and the
	// final atomic state is read at the end. We mirror that.
	var hit atomic.Bool
	c.CollisionsBox(sortedBox, true, true, func(qLeaf, lLeaf int) {
		tri0 := int(sortedFaceID[qLeaf])
		tri1 := int(sortedFaceID[lLeaf])
		var v0, v1 [3]geom.Vec3
		for k := 0; k < 3; k++ {
			v0[k] = verts[starts[3*tri0+k]]
			v1[k] = verts[starts[3*tri1+k]]
		}
		// Skip if any vert pair is within epsilon — shared vertex.
		for ii := 0; ii < 3; ii++ {
			for jj := 0; jj < 3; jj++ {
				d := v0[ii].Sub(v1[jj])
				if d.Dot(d) <= epsilonSq {
					return
				}
			}
		}
		if geom.DistanceTriangleTriangleSquared(v0, v1) != 0 {
			return
		}
		// Try ±epsilon perturbations along the OTHER tri's normal.
		var t0, t1 [3]geom.Vec3
		n0 := faceNormals[tri0]
		n1 := faceNormals[tri1]
		for k := 0; k < 3; k++ {
			t0[k] = v0[k].Add(n1.Scale(ep))
		}
		if geom.DistanceTriangleTriangleSquared(t0, v1) > 0 {
			return
		}
		for k := 0; k < 3; k++ {
			t0[k] = v0[k].Add(n1.Scale(-ep))
		}
		if geom.DistanceTriangleTriangleSquared(t0, v1) > 0 {
			return
		}
		for k := 0; k < 3; k++ {
			t1[k] = v1[k].Add(n0.Scale(ep))
		}
		if geom.DistanceTriangleTriangleSquared(v0, t1) > 0 {
			return
		}
		for k := 0; k < 3; k++ {
			t1[k] = v1[k].Add(n0.Scale(-ep))
		}
		if geom.DistanceTriangleTriangleSquared(v0, t1) > 0 {
			return
		}
		hit.Store(true)
	})
	return hit.Load()
}

// IsConvex is the Go port of C++ Manifold::Impl::IsConvex
// (src/properties.cpp). Returns true iff the mesh is genus 0 (a
// topological sphere) AND every shared edge is convex from the
// outside (the cross of the two face normals projected onto the edge
// vector is non-negative).
//
// Genus is computed from the Euler characteristic χ = V - E + F:
// genus = 1 - χ/2. For closed orientable manifolds, χ == 2(1 - g).
//
// The edge convexity check runs over every halfedge but only does
// work on the forward halfedge of each pair (matching the C++ early
// `if (!IsForward) return true` filter). C++ no-policy all_of
// defaults to threshold 1e5; we mirror.
func (i *Impl) IsConvex() bool {
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	faceNormals := i.FaceNormals()
	verts := i.Verts()
	n := len(starts)
	numTri := n / 3
	numVert := len(verts)
	numEdge := n / 2
	chi := numVert - numEdge + numTri
	genus := 1 - chi/2
	if genus != 0 {
		return false
	}
	return parallel.AllOf(parallel.AutoPolicy(n, 100000), n, func(idx int) bool {
		// IsForward(idx) ⇔ Start(idx) < End(idx).
		end := starts[nextHalfedge(idx)]
		if starts[idx] >= end {
			return true
		}
		n0 := faceNormals[idx/3]
		n1 := faceNormals[int(pairs[idx])/3]
		// Co-planar adjacent faces — trivially convex at this edge.
		if n0 == n1 {
			return true
		}
		edgeVec := verts[end].Sub(verts[starts[idx]])
		// edgeVec · (n0 × n1) > 0  ⇒ convex fold.
		return edgeVec.Dot(n0.Cross(n1)) > 0
	})
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
//
// C++ uses the no-policy `count_if` form which defaults to threshold
// 1e4 via autoPolicy. We mirror with parallel.CountIf.
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
	return parallel.CountIf(parallel.AutoPolicy(numTri, 10000), numTri, func(face int) bool {
		if pairs[3*face] < 0 {
			return true
		}
		proj := geom.GetAxisAlignedProjection(normals[face])
		var v [3]geom.Vec2
		for k := 0; k < 3; k++ {
			v[k] = proj.MulVec3(verts[starts[3*face+k]])
		}
		return geom.CCW(v[0], v[1], v[2], tol) == 0
	})
}
