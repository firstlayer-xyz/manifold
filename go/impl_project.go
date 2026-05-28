package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/orderedmap"
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

	// AssembleHalfedges (src/face_op.cpp:41-67): chain the cusps into
	// closed polygons via an ordered multimap keyed on startVert. C++
	// uses std::multimap, so begin() seeds each new polygon from the
	// LOWEST remaining startVert; within equal keys the insertion order
	// is preserved (the value slice is the equal-key bucket).
	vertEdge := orderedmap.New[int32, []int]()
	for idx, c := range cusps {
		bin, _ := vertEdge.Get(c.startVert)
		vertEdge.Set(c.startVert, append(bin, idx))
	}
	// find: the first cusp index whose startVert == key (multimap::find).
	find := func(key int32) (int, bool) {
		bin, ok := vertEdge.Get(key)
		if !ok || len(bin) == 0 {
			return -1, false
		}
		return bin[0], true
	}
	// erase: drop that first equal-key entry (multimap::erase(it)).
	erase := func(key int32) {
		bin, ok := vertEdge.Get(key)
		if !ok {
			return
		}
		if len(bin) <= 1 {
			vertEdge.Delete(key)
		} else {
			vertEdge.Set(key, bin[1:])
		}
	}

	var polys Polygons
	startIdx, thisIdx := -1, -1
	for {
		if thisIdx == startIdx {
			if vertEdge.Len() == 0 {
				break
			}
			// begin()->second: the lowest remaining startVert's first edge.
			startIdx, _ = find(vertEdge.Keys()[0])
			thisIdx = startIdx
			polys = append(polys, SimplePolygon{})
		}
		c := cusps[thisIdx]
		v := verts[c.startVert]
		polys[len(polys)-1] = append(polys[len(polys)-1], geom.Vec2{X: v.X, Y: v.Y})
		// result = find(endVert); thisEdge = result->second; erase(result).
		next, ok := find(c.endVert)
		if !ok {
			break // non-manifold silhouette (C++ DEBUG_ASSERT topologyErr)
		}
		erase(c.endVert)
		thisIdx = next
	}
	return polys
}
