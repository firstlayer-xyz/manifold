package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// calculateCurvature is the Go port of C++
// Manifold::Impl::CalculateCurvature (src/properties.cpp). Writes
// Gaussian and mean curvature into the requested property slots.
// Either index < 0 means "skip that slot."
//
// The C++ version uses atomic accumulators because for_each_n runs
// triangle work in parallel; the Go port runs the same loop
// sequentially with plain accumulators (the result is identical).
//
// Algorithm:
//
//  1. Per triangle, in the CurvatureAngles functor:
//     a. Compute the three edge vectors (normalized) and their
//        lengths.
//     b. Accumulate per-vertex mean curvature via the dihedral
//        contribution of each edge against its neighbour triangle.
//     c. Accumulate per-vertex Gaussian curvature via the interior
//        triangle-vertex angle (subtracted from 2π later).
//     d. Accumulate per-vertex Voronoi area (a third of the
//        triangle's area, distributed to its three corners).
//  2. Per vertex, normalize mean+Gaussian by (degree / (6 * area)).
//  3. Resize properties_ if needed, then write the curvature values
//     at the requested slots, preserving any previously-set extra
//     properties on the same propVert.
//
// vertPos / halfedge / faceNormal data is read from the impl through
// the existing bridge accessors; the only mutator paths are
// SetNumProp + SetProperties (already drilled).
func implCalculateCurvature(mi *bridge.MutableImpl, gaussianIdx, meanIdx int) {
	if mi.HalfedgeStartsRO() == nil {
		return // empty
	}
	if gaussianIdx < 0 && meanIdx < 0 {
		return
	}

	verts := mi.Verts()
	starts := mi.HalfedgeStartsRO()
	pairs := mi.HalfedgePairsRO()
	props := mi.HalfedgePropsRO()
	faceNormals := mi.FaceNormalsMut()
	numTri := len(starts) / 3
	numVert := len(verts)

	const twoPi = 2.0 * math.Pi
	vertMean := make([]float64, numVert)
	vertGauss := make([]float64, numVert)
	for i := range vertGauss {
		vertGauss[i] = twoPi
	}
	vertArea := make([]float64, numVert)
	degree := make([]float64, numVert)

	// Step 1: per-triangle accumulators (the CurvatureAngles loop).
	for tri := 0; tri < numTri; tri++ {
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
			if length > 0 {
				edge[i].X = dx / length
				edge[i].Y = dy / length
				edge[i].Z = dz / length
			}
			neighborTri := int(pairs[edgeIdx]) / 3
			// dihedral = 0.25 * len * asin(dot(cross(n1, n2), e))
			n1 := faceNormals[tri]
			n2 := faceNormals[neighborTri]
			cx := n1.Y*n2.Z - n1.Z*n2.Y
			cy := n1.Z*n2.X - n1.X*n2.Z
			cz := n1.X*n2.Y - n1.Y*n2.X
			d := cx*edge[i].X + cy*edge[i].Y + cz*edge[i].Z
			// asin clamps for floating-point drift outside [-1,1].
			if d > 1 {
				d = 1
			}
			if d < -1 {
				d = -1
			}
			dihedral := 0.25 * length * math.Asin(d)
			vertMean[startVert] += dihedral
			vertMean[endVert] += dihedral
			degree[startVert] += 1.0
		}

		// Interior angles (Gaussian) — guard against degenerate dot
		// values outside [-1, 1].
		var phi [3]float64
		dot20 := -(edge[2].X*edge[0].X + edge[2].Y*edge[0].Y + edge[2].Z*edge[0].Z)
		dot01 := -(edge[0].X*edge[1].X + edge[0].Y*edge[1].Y + edge[0].Z*edge[1].Z)
		if dot20 > 1 {
			dot20 = 1
		} else if dot20 < -1 {
			dot20 = -1
		}
		if dot01 > 1 {
			dot01 = 1
		} else if dot01 < -1 {
			dot01 = -1
		}
		phi[0] = math.Acos(dot20)
		phi[1] = math.Acos(dot01)
		phi[2] = math.Pi - phi[0] - phi[1]

		// Triangle area / 3 (distributed to each corner): the cross-
		// product of two edges has magnitude 2*Area, so divide by 6.
		cx := edge[0].Y*edge[1].Z - edge[0].Z*edge[1].Y
		cy := edge[0].Z*edge[1].X - edge[0].X*edge[1].Z
		cz := edge[0].X*edge[1].Y - edge[0].Y*edge[1].X
		crossLen := math.Sqrt(cx*cx + cy*cy + cz*cz)
		area3 := edgeLen[0] * edgeLen[1] * crossLen / 6

		for i := 0; i < 3; i++ {
			vert := int(starts[3*tri+i])
			vertGauss[vert] -= phi[i]
			vertArea[vert] += area3
		}
	}

	// Step 2: normalize per-vertex.
	for v := 0; v < numVert; v++ {
		if vertArea[v] > 0 {
			factor := degree[v] / (6 * vertArea[v])
			vertMean[v] *= factor
			vertGauss[v] *= factor
		} else {
			vertMean[v] = 0
			vertGauss[v] = 0
		}
	}

	// Step 3: write into properties_, preserving any existing extras
	// on each propVert.
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
	// NumPropVert when oldNumProp == 0 is numVert; else len/oldNumProp.
	numPropVert := numVert
	if oldNumProp > 0 {
		numPropVert = len(oldProperties) / oldNumProp
	}
	newProperties := make([]float64, numProp*numPropVert)

	mi.SetNumProp(numProp)

	counters := make([]uint8, numPropVert)
	for tri := 0; tri < numTri; tri++ {
		for i := 0; i < 3; i++ {
			edge := 3*tri + i
			vert := int(starts[edge])
			propVert := int(props[edge])
			if counters[propVert] == 1 {
				continue
			}
			counters[propVert] = 1
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
	}
	mi.SetProperties(newProperties)
}

