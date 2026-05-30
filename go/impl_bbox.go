package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// CalculateBBox is the Go port of C++ Manifold::Impl::CalculateBBox
// (src/properties.cpp): compute min/max of vertPos_ component-wise,
// skipping any vert whose .x is NaN (the C++ short-circuit). If the
// resulting box is non-finite (empty mesh, or all verts NaN), call
// MakeEmpty(Error::NoError) to clear the impl.
//
// C++ uses two `reduce` calls — one for min, one for max — each with
// a NaN-skipping lambda. The no-policy reduce defaults to threshold
// kSeqThreshold = 1e4 via autoPolicy. We mirror with two parallel.Reduce
// calls; both lambdas are associative+commutative (NaN propagates as
// identity) so the parallel reorder is safe.
func (mi *MutableImpl) CalculateBBox() {
	verts := mi.Verts()
	inf := math.Inf(1)
	idMin := geom.Vec3{X: inf, Y: inf, Z: inf}
	idMax := geom.Vec3{X: -inf, Y: -inf, Z: -inf}
	policy := parallel.AutoPolicy(len(verts), 10000)
	minV := parallel.Reduce(policy, verts, idMin, func(a, b geom.Vec3) geom.Vec3 {
		if math.IsNaN(a.X) {
			return b
		}
		if math.IsNaN(b.X) {
			return a
		}
		return geom.Vec3{
			X: math.Min(a.X, b.X),
			Y: math.Min(a.Y, b.Y),
			Z: math.Min(a.Z, b.Z),
		}
	})
	maxV := parallel.Reduce(policy, verts, idMax, func(a, b geom.Vec3) geom.Vec3 {
		if math.IsNaN(a.X) {
			return b
		}
		if math.IsNaN(b.X) {
			return a
		}
		return geom.Vec3{
			X: math.Max(a.X, b.X),
			Y: math.Max(a.Y, b.Y),
			Z: math.Max(a.Z, b.Z),
		}
	})
	// C++ Box::IsFinite() (common.h:434) = all(isfinite(min)) &&
	// all(isfinite(max)) — i.e. all six components finite, NOT an ordering
	// check. An empty mesh reduces to min=+inf/max=-inf (not finite); a
	// planar mesh (e.g. min.z==max.z) is still finite and must be kept.
	if !(geom.Box{Min: minV, Max: maxV}).IsFinite() {
		// Decimated out of existence - early out.
		mi.MakeEmpty(0) // Error::NoError
		return
	}
	mi.SetBBox(minV, maxV)
}
