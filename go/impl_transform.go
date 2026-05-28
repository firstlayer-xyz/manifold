package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// flipHalfedge mirrors the inline helper in src/mesh_fixes.h:
//
//	tri  = halfedge / 3
//	vert = 2 - (halfedge - 3 * tri)
//	return 3*tri + vert
//
// Reverses the position of `halfedge` within its triangle (0↔2, 1↔1).
// Used by FlipTris (negative-determinant winding fix) and the
// invert path of TransformTangents.
func flipHalfedge(h int) int {
	tri := h / 3
	v := 2 - (h - 3*tri)
	return 3*tri + v
}

// Transform is the Go port of C++ Manifold::Impl::Transform
// (src/impl.cpp). Returns a new MutableImpl with the affine 3x4
// transform applied to every position, every normal, every property
// vector (slots 0..2 when AllHaveNormals or any meshID carries
// normals), and every halfedge tangent. If det(linear part) < 0,
// winding is flipped on every triangle.
//
// The result is NOT a "fresh" Impl — it preserves the source's
// triangulation, halfedge ordering, and meshRelation_, only mutating
// the geometry to reflect the transform. C++ leaves vertPos_ /
// halfedge_ / faceNormal_ / vertNormal_ / properties_ in-place
// post-transform (no SortGeometry, no SetNormalsAndCoplanar). We
// mirror that: only CalculateBBox + epsilon scaling + Collider
// refresh.
func (i *Impl) Transform(t geom.Mat3x4) *MutableImpl {
	// Identity short-circuit. C++: `if (transform_ == mat3x4(la::identity)) return *this;`
	if isIdentityMat3x4(t) {
		return i.Copy()
	}
	result := i.Copy()
	if Error(i.Scalars().Status) != NoError {
		return result
	}
	if !mat3x4IsFinite(t) {
		result.h.MakeEmpty(int(NonFiniteVertex))
		return result
	}

	// Mark not-original; compose meshIDtransform.transform with t.
	result.h.SetMeshRelationOriginalID(-1)
	rels := result.MeshIDTransforms()
	result.h.ClearMeshIDTransforms()
	for _, r := range rels {
		composed := composeMat3x4(t, geom.Mat3x4(r.Transform))
		result.h.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID),
			[4][3]float64(composed), r.BackSide, r.HasNormals)
	}

	// Transform vertPos via Transform4x3 — parallel.Transform mirrors
	// the C++ std::transform call. The src is i.Verts() (read-only view
	// on the source impl); the dst is result.Verts() (aliased buffer on
	// the copy). Same length.
	policy := parallel.AutoPolicy(i.NumVert())
	parallel.Transform(policy, i.Verts(), result.Verts(), func(v geom.Vec3) geom.Vec3 {
		return t.ApplyAffine(v)
	})

	// Normal transform: NormalTransform(t) is inv(transpose(linear)).
	normalTransform := geom.NormalTransform(t)
	srcFN := i.FaceNormals()
	dstFN := result.FaceNormals()
	if len(srcFN) > 0 && len(dstFN) == len(srcFN) {
		parallel.Transform(policy, srcFN, dstFN, func(n geom.Vec3) geom.Vec3 {
			out := normalTransform.MulVec3(n).Normalize()
			if !out.IsFinite() {
				return geom.Vec3{}
			}
			return out
		})
	}
	srcVN := i.VertNormals()
	dstVN := result.VertNormals()
	if len(srcVN) > 0 && len(dstVN) == len(srcVN) {
		parallel.Transform(policy, srcVN, dstVN, func(n geom.Vec3) geom.Vec3 {
			out := normalTransform.MulVec3(n).Normalize()
			if !out.IsFinite() {
				return geom.Vec3{}
			}
			return out
		})
	}

	// EagerTransformPropNormals: when any meshID has normals, walk
	// halfedges and transform slot 0..2 of each propVert exactly once.
	if result.NumProp() >= 3 {
		eagerTransformPropNormals(result, normalTransform)
	}

	// Winding flip when det < 0.
	invert := mat3FromMat3x4(t).Determinant() < 0
	if invert {
		flipTris(result)
	}

	// Transform halfedge tangents (invert path uses flipHalfedge).
	srcTangents := i.HalfedgeTangents()
	if len(srcTangents) > 0 {
		linear := mat3FromMat3x4(t)
		dstTangents := append([]float64(nil), result.HalfedgeTangents()...)
		srcPairs := i.HalfedgePairs()
		n := len(srcTangents) / 4
		parallel.ForEachN(policy, n, func(edgeOut int) {
			edgeIn := edgeOut
			if invert {
				edgeIn = int(srcPairs[flipHalfedge(edgeOut)])
			}
			in := geom.Vec3{
				X: srcTangents[4*edgeIn+0],
				Y: srcTangents[4*edgeIn+1],
				Z: srcTangents[4*edgeIn+2],
			}
			out := linear.MulVec3(in)
			dstTangents[4*edgeOut+0] = out.X
			dstTangents[4*edgeOut+1] = out.Y
			dstTangents[4*edgeOut+2] = out.Z
			dstTangents[4*edgeOut+3] = srcTangents[4*edgeIn+3]
		})
		result.h.SetHalfedgeTangents(dstTangents)
	}

	// Recompute bBox and epsilon-scaling. Use the SOURCE epsilon (the
	// copy preserved it) scaled by the spectral norm — matches C++
	// `result.epsilon_ *= SpectralNorm(mat3(transform_))`.
	result.CalculateBBox()
	eps := i.Scalars().Epsilon * geom.SpectralNorm(mat3FromMat3x4(t))
	result.SetEpsilon(eps, false)

	// Refresh the C++-side Collider so unported algorithms (RayCast,
	// MinGap, Minkowski, Boolean3) see a consistent BVH. C++ avoids
	// the full rebuild via Collider::Transform or UpdateBoxes; we
	// currently rebuild because the Go-side Collider isn't persisted
	// and the bridge doesn't expose the Transform / UpdateBoxes
	// fast-paths. Functionally equivalent; less efficient.
	if result.HalfedgeCount() > 0 {
		faceBox, faceMorton := result.GetFaceBoxMorton()
		box, morton := result.SortFaces(faceBox, faceMorton)
		flat := make([]float64, 6*len(box))
		parallel.ForEachN(policy, len(box), func(idx int) {
			b := box[idx]
			flat[6*idx+0] = b.Min.X
			flat[6*idx+1] = b.Min.Y
			flat[6*idx+2] = b.Min.Z
			flat[6*idx+3] = b.Max.X
			flat[6*idx+4] = b.Max.Y
			flat[6*idx+5] = b.Max.Z
		})
		result.h.BuildCollider(flat, morton)
	}
	return result
}

// eagerTransformPropNormals is the Go port of
// Manifold::Impl::EagerTransformPropNormals (src/impl.cpp). Walks
// halfedges and, for each propVert whose meshID has hasNormals,
// transforms slot 0..2 of that propVert exactly once via
// normalTransform + SafeNormalize.
//
// Walks halfedges serially because the C++ does too (the iteration
// state — propVisited — is order-sensitive: a propVert may be
// touched multiple times across halfedges but only the first
// counts).
func eagerTransformPropNormals(mi *MutableImpl, normalTransform geom.Mat3) {
	rels := mi.MeshIDTransforms()
	relByID := make(map[int32]bool, len(rels))
	anyHasNormals := false
	for _, r := range rels {
		relByID[r.MeshID] = r.HasNormals
		if r.HasNormals {
			anyHasNormals = true
		}
	}
	if !anyHasNormals {
		return
	}

	triRefs := mi.TriRefs()
	starts := mi.HalfedgeStarts()
	halfedgeProps := mi.HalfedgeProps()
	props := append([]float64(nil), mi.Properties()...)
	numProp := mi.NumProp()
	numPropVert := len(props) / numProp
	propVisited := make([]bool, numPropVert)

	for e := 0; e < len(starts); e++ {
		tri := e / 3
		if !relByID[triRefs[tri].MeshID] {
			continue
		}
		prop := int(halfedgeProps[e])
		if prop < 0 || propVisited[prop] {
			continue
		}
		propVisited[prop] = true
		n := geom.Vec3{
			X: props[prop*numProp+0],
			Y: props[prop*numProp+1],
			Z: props[prop*numProp+2],
		}
		n = normalTransform.MulVec3(n).SafeNormalize()
		props[prop*numProp+0] = n.X
		props[prop*numProp+1] = n.Y
		props[prop*numProp+2] = n.Z
	}
	mi.h.SetProperties(props)
}

// flipTris is the Go port of FlipTris functor (src/mesh_fixes.h).
// For each tri, swap halfedges 0 and 2 within the tri AND
// flip-halfedge the paired indices, so the winding reverses.
// Mirrors the C++ per-tri body literally.
func flipTris(mi *MutableImpl) {
	oldStarts := append([]int32(nil), mi.HalfedgeStarts()...)
	oldPairs := append([]int32(nil), mi.HalfedgePairs()...)
	oldProps := append([]int32(nil), mi.HalfedgeProps()...)
	starts := make([]int32, len(oldStarts))
	pairs := make([]int32, len(oldPairs))
	props := make([]int32, len(oldProps))
	numTri := len(oldStarts) / 3
	parallel.ForEachN(parallel.AutoPolicy(numTri), numTri, func(tri int) {
		// C++ pre-swap: face = {Get(3t+2), Get(3t+1), Get(3t)}.
		// Then SetStart, SetPair, SetProp at 3*tri+i from face[i],
		// after applying flipHalfedge to face[i].pairedHalfedge AND
		// swapping start/end (the start of the swapped halfedge is
		// the original end). Since halfedge.End(e) = Start(NextHalfedge(e)),
		// face[i].endVert = oldStarts[NextHalfedge(3*tri + (2-i))].
		for i := 0; i < 3; i++ {
			srcEdge := 3*tri + (2 - i) // C++ Get(3*tri + 2 - i)
			// After swap, the new start is the original end (i.e.
			// Start(NextHalfedge(srcEdge))).
			starts[3*tri+i] = oldStarts[nextHalfedge(srcEdge)]
			pairs[3*tri+i] = int32(flipHalfedge(int(oldPairs[srcEdge])))
			props[3*tri+i] = oldProps[srcEdge]
		}
	})
	mi.h.SetHalfedgesRaw(starts, props, pairs)
}

// isIdentityMat3x4 reports whether m is the identity 3x4 affine.
func isIdentityMat3x4(m geom.Mat3x4) bool {
	return m[0][0] == 1 && m[0][1] == 0 && m[0][2] == 0 &&
		m[1][0] == 0 && m[1][1] == 1 && m[1][2] == 0 &&
		m[2][0] == 0 && m[2][1] == 0 && m[2][2] == 1 &&
		m[3][0] == 0 && m[3][1] == 0 && m[3][2] == 0
}

// mat3x4IsFinite reports whether every component is finite.
func mat3x4IsFinite(m geom.Mat3x4) bool {
	for col := 0; col < 4; col++ {
		for row := 0; row < 3; row++ {
			v := m[col][row]
			if math.IsNaN(v) || math.IsInf(v, 0) {
				return false
			}
		}
	}
	return true
}

// mat3FromMat3x4 extracts the linear 3x3 part of an affine 3x4.
func mat3FromMat3x4(m geom.Mat3x4) geom.Mat3 {
	return geom.Mat3{m[0], m[1], m[2]}
}

// composeMat3x4 returns a * b — the affine composition of two 3x4
// matrices. Matches the C++ `transform_ * Mat4(other)` pattern used
// to compose meshIDtransform.transform with a new transform.
func composeMat3x4(a, b geom.Mat3x4) geom.Mat3x4 {
	aL := mat3FromMat3x4(a)
	aT := geom.Vec3{X: a[3][0], Y: a[3][1], Z: a[3][2]}
	bL := mat3FromMat3x4(b)
	bT := geom.Vec3{X: b[3][0], Y: b[3][1], Z: b[3][2]}

	// Linear part: aL * bL (column-major Mat3 product).
	var outL geom.Mat3
	for col := 0; col < 3; col++ {
		// outL[col] = aL * bL[col] = aL.MulVec3(Vec3(bL[col]))
		bCol := geom.Vec3{X: bL[col][0], Y: bL[col][1], Z: bL[col][2]}
		o := aL.MulVec3(bCol)
		outL[col] = [3]float64{o.X, o.Y, o.Z}
	}
	// Translation: aL * bT + aT.
	outT := aL.MulVec3(bT)
	outT.X += aT.X
	outT.Y += aT.Y
	outT.Z += aT.Z
	return geom.Mat3x4{outL[0], outL[1], outL[2], [3]float64{outT.X, outT.Y, outT.Z}}
}
