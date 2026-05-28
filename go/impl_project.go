package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// implProject is the Go port of C++ Manifold::Impl::Project
// (src/face_op.cpp). It returns the XY-plane projection of the
// manifold as a set of 2D polygons (often self-intersecting; users
// typically run the result through CrossSection's positive fill rule
// to clean it up).
//
// Algorithm:
//  1. Find silhouette halfedges: edges whose own triangle faces +Z
//     (or sideways: normal.z >= 0) and whose paired triangle faces -Z
//     (normal.z < 0). Those edges trace the boundary of the projected
//     shape when viewed from +Z.
//  2. Chain the silhouette halfedges into closed polygons by linking
//     each edge's endVert to the next edge's startVert (the
//     AssembleHalfedges step in C++).
//  3. Project each vertex via the +Z axis-aligned projection
//     (drops the Z coordinate, identity-style for the X/Y components).
func implProject(impl *bridge.Impl) Polygons {
	starts := impl.HalfedgeStarts()
	pairs := impl.HalfedgePairs()
	faceNormals := impl.FaceNormals()
	verts := impl.Verts()
	if len(starts) == 0 {
		return nil
	}

	// Step 1: collect silhouette halfedges (cusps).
	type cusp struct{ startVert, endVert int32 }
	cusps := make([]cusp, 0, len(starts)/4)
	for i := 0; i < len(starts); i++ {
		pair := int(pairs[i])
		// pair-of-pair == i, so faceNormals[i/3] is this tri's normal.
		if faceNormals[i/3].Z >= 0 && faceNormals[pair/3].Z < 0 {
			cusps = append(cusps, cusp{
				startVert: starts[i],
				endVert:   starts[nextHalfedge(i)],
			})
		}
	}
	if len(cusps) == 0 {
		return nil
	}

	// Step 2: AssembleHalfedges — chain cusps into closed polygons.
	// Build a multimap startVert → cusp-index for O(1) lookup. The
	// C++ uses std::multimap; for our small inputs a slice-of-indices
	// per vertex is fine.
	vertEdge := make(map[int32][]int, len(cusps))
	for i, c := range cusps {
		vertEdge[c.startVert] = append(vertEdge[c.startVert], i)
	}
	// take consumes one entry for startVert and returns its cusp idx.
	take := func(startVert int32) (int, bool) {
		bin, ok := vertEdge[startVert]
		if !ok || len(bin) == 0 {
			return -1, false
		}
		idx := bin[0]
		if len(bin) == 1 {
			delete(vertEdge, startVert)
		} else {
			vertEdge[startVert] = bin[1:]
		}
		return idx, true
	}

	var polys Polygons
	// Walk: take any remaining cusp as the start; follow endVert →
	// startVert chain until we close the loop.
	for len(vertEdge) > 0 {
		var anyStart int32
		for k := range vertEdge {
			anyStart = k
			break
		}
		startIdx, _ := take(anyStart)
		thisIdx := startIdx
		var poly SimplePolygon
		for {
			c := cusps[thisIdx]
			// Step 3 (project + emit) — projection for +Z normal is
			// identity on (x,y), so just drop z.
			v := verts[c.startVert]
			poly = append(poly, geom.Vec2{X: v.X, Y: v.Y})
			nextIdx, ok := take(c.endVert)
			if !ok {
				// non-manifold or closed: loop back to start
				break
			}
			thisIdx = nextIdx
			if thisIdx == startIdx {
				break
			}
		}
		polys = append(polys, poly)
	}
	return polys
}
