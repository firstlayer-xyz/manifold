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

// sharpenEdges is the Go port of C++ Manifold::Impl::SharpenEdges
// (src/smoothing.cpp). Returns a list of (halfedge, smoothness)
// entries for every halfedge whose dihedral (the angle between its
// face and its pair's face) exceeds minSharpAngle degrees. The
// caller-supplied angle is floored at kMinSharpAngle.
//
// Both ends of each detected sharp edge are emitted, so the returned
// list has even length: positions 2i and 2i+1 are the matching
// forward/backward halfedges.
func implSharpenEdges(impl *bridge.Impl, minSharpAngle, minSmoothness float64) []bridge.Smoothness {
	if minSharpAngle < kMinSharpAngle {
		minSharpAngle = kMinSharpAngle
	}
	minRadians := minSharpAngle * (math.Pi / 180.0)

	starts := impl.HalfedgeStarts()
	pairs := impl.HalfedgePairs()
	faceNormals := impl.FaceNormals()
	var out []bridge.Smoothness
	for e := 0; e < len(starts); e++ {
		// IsForward: Start(e) < End(e).
		startV := starts[e]
		endV := starts[nextHalfedge(e)]
		if startV >= endV {
			continue
		}
		pair := int(pairs[e])
		// AngleBetween(n1, n2) = acos(clamp(dot, -1, 1)) — the C++ uses
		// la::angleBetween which clamps internally.
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
