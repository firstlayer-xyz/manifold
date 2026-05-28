package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// CalculateBBox is the Go port of C++ Manifold::Impl::CalculateBBox
// (src/properties.cpp): compute min/max of vertPos_ component-wise,
// skipping any vert whose .x is NaN (the C++ short-circuit). If the
// resulting box is non-finite (empty mesh, or all verts NaN), call
// MakeEmpty(Error::NoError) to clear the impl.
//
// Faithful to C++: when one operand has NaN x, the other operand wins
// the reduce. This matches the la::min / la::max behavior in the C++
// lambda.
func (mi *MutableImpl) CalculateBBox() {
	verts := mi.Verts()
	inf := math.Inf(1)
	minV := geom.Vec3{X: inf, Y: inf, Z: inf}
	maxV := geom.Vec3{X: -inf, Y: -inf, Z: -inf}
	for _, v := range verts {
		if math.IsNaN(v.X) {
			continue
		}
		if math.IsNaN(minV.X) {
			minV = v
		} else {
			if v.X < minV.X {
				minV.X = v.X
			}
			if v.Y < minV.Y {
				minV.Y = v.Y
			}
			if v.Z < minV.Z {
				minV.Z = v.Z
			}
		}
		if math.IsNaN(maxV.X) {
			maxV = v
		} else {
			if v.X > maxV.X {
				maxV.X = v.X
			}
			if v.Y > maxV.Y {
				maxV.Y = v.Y
			}
			if v.Z > maxV.Z {
				maxV.Z = v.Z
			}
		}
	}
	// Box::IsFinite check from common.h: min.x < max.x && min.y < max.y
	// && min.z < max.z. A degenerate (empty or all-NaN) impl never
	// satisfies this; mirror the C++ early-out via MakeEmpty(NoError=0).
	finite := minV.X < maxV.X && minV.Y < maxV.Y && minV.Z < maxV.Z
	if !finite {
		mi.h.MakeEmpty(0) // Error::NoError
		return
	}
	mi.h.SetBBox(minV, maxV)
}
