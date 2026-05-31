package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// kMinSharpAngle mirrors the C++ constexpr in src/smoothing.cpp.
// Below this dihedral the floating-point noise in the cross/asin
// computation overwhelms the signal, so SharpenEdges floors the
// caller-supplied angle at this value.
const kMinSharpAngle = 5.0

// UpdateSharpenedEdges is the Go port of Manifold::Impl::UpdateSharpenedEdges
// (src/smoothing.cpp:346): remap user-supplied sharpened-edge halfedge indices
// (referenced to the input MeshGL's triangles, recorded as triRef.faceID) onto
// this Impl's post-sort halfedge indices. Used by Smooth(MeshGL).
func (mi *MutableImpl) UpdateSharpenedEdges(sharpenedEdges []Smoothness) []bridge.Smoothness {
	oldHalfedge2New := make(map[int]int)
	refs := mi.TriRefs()
	for tri := 0; tri < mi.NumTri(); tri++ {
		oldTri := int(refs[tri].FaceID)
		for i := 0; i < 3; i++ {
			oldHalfedge2New[3*oldTri+i] = 3*tri + i
		}
	}
	newSharp := make([]bridge.Smoothness, len(sharpenedEdges))
	for k, edge := range sharpenedEdges {
		newSharp[k] = bridge.Smoothness{
			Halfedge:   uint64(oldHalfedge2New[int(edge.Halfedge)]),
			Smoothness: edge.Smoothness,
		}
	}
	return newSharp
}

// SharpenEdges is the Go port of C++ Manifold::Impl::SharpenEdges
// (src/smoothing.cpp). Returns a list of (halfedge, smoothness)
// entries for every halfedge whose dihedral (the angle between its
// face and its pair's face) exceeds minSharpAngle degrees. The
// caller-supplied angle is floored at kMinSharpAngle.
//
// Both ends of each detected sharp edge are emitted, so the returned
// list has even length: positions 2i and 2i+1 are the matching
// forward/backward halfedges.
func (i *Impl) SharpenEdges(minSharpAngle, minSmoothness float64) []bridge.Smoothness {
	if minSharpAngle < kMinSharpAngle {
		minSharpAngle = kMinSharpAngle
	}
	minRadians := minSharpAngle * (math.Pi / 180.0)

	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	faceNormals := i.FaceNormals()
	var out []bridge.Smoothness
	for e := 0; e < len(starts); e++ {
		startV := starts[e]
		endV := starts[nextHalfedge(e)]
		if startV >= endV {
			continue
		}
		pair := int(pairs[e])
		n1 := faceNormals[e/3]
		n2 := faceNormals[pair/3]
		d := n1.X*n2.X + n1.Y*n2.Y + n1.Z*n2.Z
		if d > 1 {
			d = 1
		} else if d < -1 {
			d = -1
		}
		dihedral := math.Acos(d)
		if dihedral > minRadians {
			out = append(out, bridge.Smoothness{Halfedge: uint64(e), Smoothness: minSmoothness})
			out = append(out, bridge.Smoothness{Halfedge: uint64(pair), Smoothness: minSmoothness})
		}
	}
	return out
}
