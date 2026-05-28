package manifold

import "github.com/firstlayer-xyz/manifold/go/internal/geom"

// Warp is the Go port of C++ Manifold::Impl::Warp (src/impl.cpp).
// Applies the user's per-vert callback to every position in vertPos_,
// then re-runs the finalization tail. C++ implements Warp as a
// WarpBatch wrapper that does a sequential for_each over the verts;
// we mirror that exactly — the per-vert callback may not be
// goroutine-safe, so we never parallelize it. Use WarpBatch with a
// parallel-safe callback if parallelism is wanted.
func (mi *MutableImpl) Warp(fn func(*geom.Vec3)) {
	mi.WarpBatch(func(verts []geom.Vec3) {
		for i := range verts {
			fn(&verts[i])
		}
	})
}

// WarpBatch is the Go port of C++ Manifold::Impl::WarpBatch
// (src/impl.cpp). Hands the user the full vertPos_ slice in one call,
// then re-runs the finalization tail (CalculateBBox → finiteness
// check → SetEpsilon → SortGeometry → SetNormalsAndCoplanar) and
// clears the originalID (warped manifolds are no longer originals).
//
// Mirrors C++ step for step. The bridge Warp/WarpBatch calls are
// gone; everything is Go-native.
func (mi *MutableImpl) WarpBatch(fn func([]geom.Vec3)) {
	fn(mi.Verts())
	mi.CalculateBBox()
	// C++ checks Impl::IsFinite() (every vert finite). Any non-finite
	// vert that survives CalculateBBox (which only clears state when
	// the ENTIRE bbox is non-finite — i.e. all verts NaN) triggers
	// NonFiniteVertex here.
	if !mi.IsFinite() {
		mi.h.MakeEmpty(int(NonFiniteVertex))
		return
	}
	mi.SetEpsilon(-1, false)
	mi.SortGeometry()
	mi.SetNormalsAndCoplanar()
	// Warped manifolds are no longer originals.
	mi.h.SetMeshRelationOriginalID(-1)
}
