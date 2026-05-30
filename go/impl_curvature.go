package manifold

import (
	"math"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// CalculateCurvature is the Go port of C++
// Manifold::Impl::CalculateCurvature (src/properties.cpp). Writes
// Gaussian and mean curvature into the requested property slots.
// Either index < 0 means "skip that slot."
//
// Three parallel passes, mirroring the C++:
//
//  1. for_each over NumTri() with CurvatureAngles functor body.
//     Per-vert accumulators (meanCurvature, gaussianCurvature,
//     area, degree) are written via AtomicAdd because multiple
//     triangles touch the same vert.
//  2. for_each_n over NumVert(): normalize mean+Gaussian by
//     (degree / (6 * area)). Each iteration writes a unique vert.
//  3. for_each_n over NumTri(): write properties_. Each tri claims
//     its propVerts via atomic_exchange on counters[propVert] — the
//     first tri to touch a propVert wins and writes; later tris
//     skip. C++ uses atomic_exchange<uint8_t>; we use uint32 (Go's
//     smallest atomic) with the same swap-to-1 semantic.
//
// C++ policy = autoPolicy(NumTri(), 1e4). We mirror.
func (mi *MutableImpl) CalculateCurvature(gaussianIdx, meanIdx int) {
	if mi.NumTri() == 0 {
		return // C++ `if (IsEmpty()) return;` where IsEmpty() == NumTri()==0
	}
	if gaussianIdx < 0 && meanIdx < 0 {
		return
	}

	verts := mi.Verts()
	starts := mi.HalfedgeStarts()
	pairs := mi.HalfedgePairs()
	props := mi.HalfedgeProps()
	faceNormals := mi.FaceNormals()
	numTri := len(starts) / 3
	numVert := len(verts)
	policy := parallel.AutoPolicy(numTri, 10000)

	const twoPi = 2.0 * math.Pi
	vertMean := make([]float64, numVert)
	vertGauss := make([]float64, numVert)
	parallel.Fill(policy, vertGauss, twoPi)
	vertArea := make([]float64, numVert)
	degree := make([]float64, numVert)

	// Pass 1: CurvatureAngles functor body, parallel over triangles.
	// Per-vert accumulators use AtomicAddFloat64.
	parallel.ForEachN(policy, numTri, func(tri int) {
		var edge [3]struct{ X, Y, Z float64 }
		var edgeLen [3]float64
		for i := 0; i < 3; i++ {
			edgeIdx := 3*tri + i
			startVert := int(starts[edgeIdx])
			endVert := int(starts[nextHalfedge(edgeIdx)])
			dx := verts[endVert].X - verts[startVert].X
			dy := verts[endVert].Y - verts[startVert].Y
			dz := verts[endVert].Z - verts[startVert].Z
			length := math.Sqrt(dx*dx + dy*dy + dz*dz)
			edgeLen[i] = length
			// C++ divides unconditionally (properties.cpp:46
			// `edge[i] /= edgeLength[i]`): a zero-length (degenerate) edge
			// yields NaN components, matching C++. No guard — adding one
			// would diverge on degenerate input.
			edge[i].X = dx / length
			edge[i].Y = dy / length
			edge[i].Z = dz / length
			neighborTri := int(pairs[edgeIdx]) / 3
			n1 := faceNormals[tri]
			n2 := faceNormals[neighborTri]
			cx := n1.Y*n2.Z - n1.Z*n2.Y
			cy := n1.Z*n2.X - n1.X*n2.Z
			cz := n1.X*n2.Y - n1.Y*n2.X
			// C++ calls math::asin directly with no clamp
			// (src/properties.cpp:50). Match it — do not clamp d.
			d := cx*edge[i].X + cy*edge[i].Y + cz*edge[i].Z
			dihedral := 0.25 * length * math.Asin(d)
			parallel.AtomicAddFloat64(&vertMean[startVert], dihedral)
			parallel.AtomicAddFloat64(&vertMean[endVert], dihedral)
			parallel.AtomicAddFloat64(&degree[startVert], 1.0)
		}

		// C++ calls math::acos directly with no clamp
		// (src/properties.cpp:58-59). Match it — do not clamp the dots.
		var phi [3]float64
		dot20 := -(edge[2].X*edge[0].X + edge[2].Y*edge[0].Y + edge[2].Z*edge[0].Z)
		dot01 := -(edge[0].X*edge[1].X + edge[0].Y*edge[1].Y + edge[0].Z*edge[1].Z)
		phi[0] = math.Acos(dot20)
		phi[1] = math.Acos(dot01)
		phi[2] = math.Pi - phi[0] - phi[1]

		cx := edge[0].Y*edge[1].Z - edge[0].Z*edge[1].Y
		cy := edge[0].Z*edge[1].X - edge[0].X*edge[1].Z
		cz := edge[0].X*edge[1].Y - edge[0].Y*edge[1].X
		crossLen := math.Sqrt(cx*cx + cy*cy + cz*cz)
		area3 := edgeLen[0] * edgeLen[1] * crossLen / 6

		for i := 0; i < 3; i++ {
			vert := int(starts[3*tri+i])
			parallel.AtomicAddFloat64(&vertGauss[vert], -phi[i])
			parallel.AtomicAddFloat64(&vertArea[vert], area3)
		}
	})

	// Pass 2: normalize per-vert.
	parallel.ForEachN(policy, numVert, func(v int) {
		factor := degree[v] / (6 * vertArea[v])
		vertMean[v] *= factor
		vertGauss[v] *= factor
	})

	oldNumProp := mi.NumProp()
	maxIdx := gaussianIdx
	if meanIdx > maxIdx {
		maxIdx = meanIdx
	}
	numProp := oldNumProp
	if maxIdx+1 > numProp {
		numProp = maxIdx + 1
	}
	oldProperties := append([]float64(nil), mi.Properties()...)
	numPropVert := numVert
	if oldNumProp > 0 {
		numPropVert = len(oldProperties) / oldNumProp
	}
	newProperties := make([]float64, numProp*numPropVert)
	mi.SetNumProp(numProp)

	// Pass 3: claim each propVert via atomic-exchange counter.
	counters := make([]uint32, numPropVert)
	parallel.ForEachN(policy, numTri, func(tri int) {
		for i := 0; i < 3; i++ {
			edge := 3*tri + i
			vert := int(starts[edge])
			propVert := int(props[edge])
			old := atomic.SwapUint32(&counters[propVert], 1)
			if old == 1 {
				continue
			}
			for p := 0; p < oldNumProp; p++ {
				newProperties[numProp*propVert+p] = oldProperties[oldNumProp*propVert+p]
			}
			if gaussianIdx >= 0 {
				newProperties[numProp*propVert+gaussianIdx] = vertGauss[vert]
			}
			if meanIdx >= 0 {
				newProperties[numProp*propVert+meanIdx] = vertMean[vert]
			}
		}
	})
	mi.SetProperties(newProperties)
}
