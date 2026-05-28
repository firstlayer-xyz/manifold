package manifold

import "math"

// Quality controls the tessellation density of factories that
// approximate smooth curves (Sphere, Cylinder, Revolve, CircularSegments).
// Mirrors the C++ Quality static class in include/manifold/common.h.
//
// Three knobs (with the same precedence as C++):
//   1. SetCircularSegments(n>0) — exact override
//   2. minimum angle per segment (default 10°)
//   3. minimum edge length (default 1.0)
//
// When SetCircularSegments has not been called (the default), the
// effective segment count for a given radius is:
//   nSeg = min(360/angle, 2π·|radius|/length) + 3, rounded down to a
//   multiple of 4, with a floor of 4.

// Defaults match the C++ constexprs in include/manifold/common.h.
const (
	qualityDefaultSegments = 0
	qualityDefaultAngle    = 10.0
	qualityDefaultLength   = 1.0
)

// Package-level Quality state. The C++ side keeps its own separate
// statics; tests that mutate Quality from C++ via the public C API do
// not affect this Go state and vice versa. Calls to Quality from Go-
// port code (Sphere, Cylinder, Revolve, CircularSegments) read these
// values.
var (
	qualitySegments       = qualityDefaultSegments
	qualityCircularAngle  = qualityDefaultAngle
	qualityEdgeLengthMin  = qualityDefaultLength
)

// SetMinCircularAngle sets the minimum angle (in degrees) between
// adjacent segments for circular approximations. Values <= 0 are
// ignored. Mirrors Quality::SetMinCircularAngle.
func SetMinCircularAngle(angle float64) {
	if angle <= 0 {
		return
	}
	qualityCircularAngle = angle
}

// SetMinCircularEdgeLength sets the minimum edge length for circular
// approximations. Values <= 0 are ignored. Mirrors
// Quality::SetMinCircularEdgeLength.
func SetMinCircularEdgeLength(length float64) {
	if length <= 0 {
		return
	}
	qualityEdgeLengthMin = length
}

// SetCircularSegments overrides the default segment count for
// circular approximations. Values < 3 (except 0, which clears the
// override) are ignored. Mirrors Quality::SetCircularSegments.
func SetCircularSegments(n int) {
	if n < 3 && n != 0 {
		return
	}
	qualitySegments = n
}

// GetCircularSegments returns the number of segments that should be
// used for a circle/sphere of the given radius under the current
// Quality settings. Mirrors Quality::GetCircularSegments.
func GetCircularSegments(radius float64) int {
	if qualitySegments > 0 {
		return qualitySegments
	}
	nSegA := int(360.0 / qualityCircularAngle)
	nSegL := int(2.0 * math.Abs(radius) * math.Pi / qualityEdgeLengthMin)
	nSeg := nSegA
	if nSegL < nSegA {
		nSeg = nSegL
	}
	nSeg += 3
	nSeg -= nSeg % 4
	if nSeg < 4 {
		nSeg = 4
	}
	return nSeg
}

// ResetQualityToDefaults restores Quality settings to their defaults.
// Mirrors Quality::ResetToDefaults.
func ResetQualityToDefaults() {
	qualitySegments = qualityDefaultSegments
	qualityCircularAngle = qualityDefaultAngle
	qualityEdgeLengthMin = qualityDefaultLength
}
