package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/triangulate"
)

// triangulateNative triangulates a set of polygons, using the native convex
// fast path (internal/triangulate) when the input is convex and falling back
// to the bridge for the concave (EarClip) case until it is drilled. Returns
// flat triangle indices (3 per triangle), matching bridge.Triangulate.
// Mirrors C++ Triangulate (src/polygon.cpp:996) with allowConvex=true.
//
// Not yet wired into extrude/revolve — that call-site switch lands once the
// EarClip path is complete (Triangulate increment 8).
func triangulateNative(polygons [][]geom.Vec2, epsilon float64) []int32 {
	if tris, ok := triangulate.Triangulate(polygons, epsilon, true); ok {
		flat := make([]int32, 0, 3*len(tris))
		for _, t := range tris {
			flat = append(flat, int32(t[0]), int32(t[1]), int32(t[2]))
		}
		return flat
	}
	return bridge.Triangulate(polygons, epsilon)
}
