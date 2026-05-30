package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// Minkowski is the Go port of Manifold::Impl::Minkowski (src/minkowski.cpp:27):
// the Minkowski sum (dilate) or, when inset is true, difference (erode) of two
// meshes. The C++ builds a CSG batch of per-feature convex hulls and unions
// (sum) or subtracts (difference) them from a copy of the base mesh.
//
// The C++ evalBatch forces ctx-observed lazy evaluation; Go's BatchBoolean is
// already eager (and has no cancellation context), so evalBatch collapses to a
// plain BatchBoolean call.
func (i *Impl) Minkowski(other *Impl, inset bool) *Manifold {
	aImpl, bImpl := i, other
	aConvex := aImpl.IsConvex()
	bConvex := bImpl.IsConvex()

	const batchSize = 1000

	// If the convex manifold was supplied first, swap them.
	if aConvex && !bConvex {
		aImpl, bImpl = bImpl, aImpl
		aConvex, bConvex = bConvex, aConvex
	}

	// Early-exit if either input is empty (Impl::IsEmpty == NumTri()==0).
	if bImpl.NumTri() == 0 {
		return aImpl.Copy().ToManifold()
	}
	if aImpl.NumTri() == 0 {
		return bImpl.Copy().ToManifold()
	}

	aVerts := aImpl.Verts()
	bVerts := bImpl.Verts()
	aStarts := aImpl.HalfedgeStarts()

	// composedHulls[0] is a copy of the base mesh; the per-feature hulls follow.
	composedHulls := []*Manifold{aImpl.Copy().ToManifold()}

	switch {
	case !inset && aConvex && bConvex:
		// Convex-Convex: one hull of every (aVert + bVert) sum. Fast.
		simpleHull := make([]Vec3, 0, len(bVerts)*len(aVerts))
		for _, vertex := range aVerts {
			for _, v := range bVerts {
				simpleHull = append(simpleHull, v.Add(vertex))
			}
		}
		composedHulls = append(composedHulls, HullPts(simpleHull))

	case (inset || !aConvex) && bConvex:
		// Convex - Non-Convex: one hull per A face (its 3 verts swept by all of
		// B's verts), unioned in batches of batchSize. Slower.
		numTri := aImpl.NumTri()
		for offset := 0; offset < numTri; offset += batchSize {
			numIter := numTri - offset
			if numIter > batchSize {
				numIter = batchSize
			}
			newHulls := make([]*Manifold, numIter)
			parallel.ForEachN(parallel.AutoPolicy(numIter, 100), numIter, func(iter int) {
				var simpleHull []Vec3
				for k := 0; k < 3; k++ {
					edge := (offset+iter)*3 + k
					vertex := aVerts[aStarts[edge]]
					for _, v := range bVerts {
						simpleHull = append(simpleHull, v.Add(vertex))
					}
				}
				newHulls[iter] = HullPts(simpleHull)
			})
			composedHulls = append(composedHulls, BatchBoolean(newHulls, OpAdd))
		}

	case !aConvex && !bConvex:
		// Non-Convex - Non-Convex: per A-face, hull each non-coplanar B face's
		// 9 vertex sums; union per A face; periodically reduce the accumulator to
		// bound memory. Very slow.
		numTriA := aImpl.NumTri()
		numTriB := bImpl.NumTri()
		const reduceThreshold = 200
		bStarts := bImpl.HalfedgeStarts()
		aFaceNormals := aImpl.FaceNormals()
		bFaceNormals := bImpl.FaceNormals()

		var accumulated []*Manifold
		for aFace := 0; aFace < numTriA; aFace++ {
			a1 := aVerts[aStarts[aFace*3+0]]
			a2 := aVerts[aStarts[aFace*3+1]]
			a3 := aVerts[aStarts[aFace*3+2]]
			nA := aFaceNormals[aFace]

			faceHulls := make([]*Manifold, numTriB)
			parallel.ForEachN(parallel.AutoPolicy(numTriB, 100), numTriB, func(bFace int) {
				const kCoplanarTol = 1e-12
				nB := bFaceNormals[bFace]
				dotSame := nA.Dot(nB)
				dotOpp := nA.Dot(nB.Scale(-1))
				coplanar := math.Abs(dotSame-1.0) < kCoplanarTol ||
					math.Abs(dotOpp-1.0) < kCoplanarTol
				if coplanar {
					return
				}
				b1 := bVerts[bStarts[bFace*3+0]]
				b2 := bVerts[bStarts[bFace*3+1]]
				b3 := bVerts[bStarts[bFace*3+2]]
				faceHulls[bFace] = HullPts([]Vec3{
					a1.Add(b1), a1.Add(b2), a1.Add(b3),
					a2.Add(b1), a2.Add(b2), a2.Add(b3),
					a3.Add(b1), a3.Add(b2), a3.Add(b3),
				})
			})

			var validFaceHulls []*Manifold
			for _, hull := range faceHulls {
				if hull != nil && !hull.IsEmpty() {
					validFaceHulls = append(validFaceHulls, hull)
				}
			}
			if len(validFaceHulls) > 0 {
				accumulated = append(accumulated, BatchBoolean(validFaceHulls, OpAdd))
			}
			if len(accumulated) >= reduceThreshold {
				reduced := BatchBoolean(accumulated, OpAdd)
				accumulated = []*Manifold{reduced}
			}
		}
		if len(accumulated) > 0 {
			composedHulls = append(composedHulls, BatchBoolean(accumulated, OpAdd))
		}
	}

	op := OpAdd
	if inset {
		op = OpSubtract
	}
	return BatchBoolean(composedHulls, op).AsOriginal()
}
