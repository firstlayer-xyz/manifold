package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// Project is the Go port of C++ Manifold::Impl::Project
// (src/face_op.cpp). Returns the XY-plane projection of the manifold
// as a set of 2D polygons (often self-intersecting; users typically
// run the result through CrossSection's positive fill rule to clean
// it up).
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
func (i *Impl) Project() Polygons {
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	faceNormals := i.FaceNormals()
	verts := i.Verts()
	if len(starts) == 0 {
		return nil
	}

	type cusp struct{ startVert, endVert int32 }
	cusps := make([]cusp, 0, len(starts)/4)
	for k := 0; k < len(starts); k++ {
		pair := int(pairs[k])
		if faceNormals[k/3].Z >= 0 && faceNormals[pair/3].Z < 0 {
			cusps = append(cusps, cusp{
				startVert: starts[k],
				endVert:   starts[nextHalfedge(k)],
			})
		}
	}
	if len(cusps) == 0 {
		return nil
	}

	vertEdge := make(map[int32][]int, len(cusps))
	for idx, c := range cusps {
		vertEdge[c.startVert] = append(vertEdge[c.startVert], idx)
	}
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
			v := verts[c.startVert]
			poly = append(poly, geom.Vec2{X: v.X, Y: v.Y})
			nextIdx, ok := take(c.endVert)
			if !ok {
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
