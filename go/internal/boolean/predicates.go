// Package boolean is the Go port of the C++ manifold CSG/Boolean engine
// (src/boolean3.cpp, src/boolean_result.cpp). Like internal/triangulate, it
// operates on raw mesh arrays passed in — no dependency on the manifold facade
// package — so the cgo bridge stays out of the kernel.
//
// The Boolean is NOT exact arithmetic: it uses symbolic perturbation via float
// predicates (Shadows/withSign/Interpolate). Correctness hinges on transcribing
// the float-expression order byte-for-byte, so these foundational predicates
// are ported and unit-tested first.
package boolean

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// Next3 / Prev3 are the Go ports of Next3 / Prev3 (src/utils.h:41/46): the next
// / previous index within a triangle, via the same lookup tables.
func Next3(i int) int { return [3]int{1, 2, 0}[i] }
func Prev3(i int) int { return [3]int{2, 0, 1}[i] }

// withSign is the Go port of withSign (src/shared.h:66): v if pos, else -v.
func withSign(pos bool, v float64) float64 {
	if pos {
		return v
	}
	return -v
}

// shadows is the Go port of Shadows (src/shared.h:94): `p < q` with symbolic
// perturbation — when p == q exactly, dir < 0 is the tiebreaker. The exact ==
// comparison is load-bearing (it drives the perturbation tiebreaks).
func shadows(p, q, dir float64) bool {
	if p == q {
		return dir < 0
	}
	return p < q
}

// interpolate is the Go port of Interpolate (src/shared.h:73): the (y, z) of
// segment aL-aR at the given x. The (x - aL) vs (x - aR) choice takes the
// smaller magnitude to keep FP error low near either endpoint; on a non-finite
// lambda or dLR it falls back to aL's (y, z). The returned Vec2 is (y, z).
func interpolate(aL, aR geom.Vec3, x float64) geom.Vec2 {
	dxL := x - aL.X
	dxR := x - aR.X
	// C++ DEBUG_ASSERT(dxL*dxR <= 0): domain check, debug-only.
	useL := math.Abs(dxL) < math.Abs(dxR)
	dLR := aR.Sub(aL)
	num := dxR
	if useL {
		num = dxL
	}
	lambda := num / dLR.X
	if !isFinite(lambda) || !isFinite(dLR.Y) || !isFinite(dLR.Z) {
		return geom.Vec2{X: aL.Y, Y: aL.Z}
	}
	baseY, baseZ := aR.Y, aR.Z
	if useL {
		baseY, baseZ = aL.Y, aL.Z
	}
	return geom.Vec2{
		X: lambda*dLR.Y + baseY,
		Y: lambda*dLR.Z + baseZ,
	}
}

// isFinite mirrors std::isfinite for a scalar.
func isFinite(x float64) bool {
	return !math.IsInf(x, 0) && !math.IsNaN(x)
}
