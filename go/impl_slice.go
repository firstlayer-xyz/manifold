package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// slice is the Go port of C++ Manifold::Impl::Slice
// (src/face_op.cpp): cut the manifold with the horizontal plane z =
// height and return the resulting closed polygons.
//
// The C++ version uses the Collider's bounding-box query to narrow
// down candidate triangles; this Go port iterates triangles directly
// (O(numTri)) since we haven't ported the Collider yet. The
// candidate-set semantics — "include a triangle iff its z-range
// straddles height (min ≤ height < max)" — are preserved exactly.
//
// The polygon-walking step is a faithful port: starting from any
// candidate triangle, find the first halfedge whose start is above
// the plane and whose end is on/below, then walk Pair-Next3 around
// the boundary, recording the interpolated intersection point on
// each crossing edge. Each closed loop becomes one SimplePolygon.
func implSlice(impl *bridge.Impl, height float64) Polygons {
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	pairs := impl.HalfedgePairs()
	numTri := len(starts) / 3

	// Step 1 (Collider replacement): pick triangles whose z-range
	// straddles the plane. Matches the C++ recordCollision filter.
	tris := make(map[int]struct{})
	for tri := 0; tri < numTri; tri++ {
		minZ, maxZ := verts[starts[3*tri]].Z, verts[starts[3*tri]].Z
		for j := 1; j < 3; j++ {
			z := verts[starts[3*tri+j]].Z
			if z < minZ {
				minZ = z
			}
			if z > maxZ {
				maxZ = z
			}
		}
		if minZ <= height && maxZ > height {
			tris[tri] = struct{}{}
		}
	}

	// Step 2: walk boundary loops.
	var polys Polygons
	for len(tris) > 0 {
		var startTri int
		for tri := range tris {
			startTri = tri
			break
		}
		// Find the first edge whose start is above the plane and whose
		// next vertex is on/below — that's the "up" edge to start from.
		k := 0
		for j := 0; j < 3; j++ {
			nextJ := j + 1
			if nextJ == 3 {
				nextJ = 0
			}
			zJ := verts[starts[3*startTri+j]].Z
			zNext := verts[starts[3*startTri+nextJ]].Z
			if zJ > height && zNext <= height {
				k = nextJ
				break
			}
		}

		var poly SimplePolygon
		tri := startTri
		for {
			delete(tris, tri)
			// halfedge_.End(edge) = halfedge_.Start(NextHalfedge(edge)).
			edge := 3*tri + k
			endStart := starts[nextHalfedge(edge)]
			if verts[endStart].Z <= height {
				k = (k + 1) % 3
			}
			up := 3*tri + k
			below := verts[starts[up]]
			above := verts[starts[nextHalfedge(up)]]
			a := (height - below.Z) / (above.Z - below.Z)
			poly = append(poly, geom.Vec2{
				X: below.X + a*(above.X-below.X),
				Y: below.Y + a*(above.Y-below.Y),
			})

			pair := int(pairs[up])
			tri = pair / 3
			k = (pair%3 + 1) % 3
			if tri == startTri {
				break
			}
		}
		polys = append(polys, poly)
	}
	return polys
}
