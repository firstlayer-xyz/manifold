package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// kPrecision mirrors the C++ constexpr in src/utils.h.
const kPrecision = 1e-12

// fltEpsilon is std::numeric_limits<float>::epsilon() (= 2^-23). Used
// by SetEpsilon's useSingle branch as the float-precision floor.
const fltEpsilon = 1.0 / (1 << 23)

// maxEpsilon is the Go port of the inline MaxEpsilon helper
// (src/shared.h): epsilon = max(minEpsilon, kPrecision * bBox.Scale()),
// returning -1 when the result is non-finite (matching C++).
func maxEpsilon(minEpsilon float64, bBox geom.Box) float64 {
	eps := math.Max(minEpsilon, kPrecision*bBox.Scale())
	if math.IsInf(eps, 0) || math.IsNaN(eps) {
		return -1
	}
	return eps
}

// SetEpsilon is the Go port of C++ Manifold::Impl::SetEpsilon
// (src/impl.cpp). The C++ signature is
// `SetEpsilon(double minEpsilon = -1, bool useSingle = false)`;
// the Go method takes both parameters explicitly. Pass minEpsilon=-1
// for the no-floor default; useSingle=true enables the
// float-precision tolerance floor (used by GetMeshGL<float>).
func (mi *MutableImpl) SetEpsilon(minEpsilon float64, useSingle bool) {
	minV, maxV := mi.BBox()
	bBox := geom.Box{Min: minV, Max: maxV}
	eps := maxEpsilon(minEpsilon, bBox)
	mi.SetEpsilonValue(eps)
	minTol := eps
	if useSingle {
		minTol = math.Max(minTol, fltEpsilon*bBox.Scale())
	}
	if cur := mi.Tolerance(); cur > minTol {
		minTol = cur
	}
	mi.SetToleranceValue(minTol)
}
