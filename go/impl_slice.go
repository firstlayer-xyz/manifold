package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// Slice is the Go port of C++ Manifold::Impl::Slice (src/face_op.cpp):
// cut the manifold with the horizontal plane z = height and return
// the resulting closed polygons.
//
// C++ uses the Collider's bounding-box query to filter candidate
// triangles whose AABB straddles the plane. We mirror via an
// ephemerally-built Go Collider over per-face boxes + Morton codes
// from GetFaceBoxMorton. The polygon-walking step is a faithful
// port: from any candidate, find the first edge whose start is
// above the plane and whose end is on/below, then walk Pair·Next3
// around the boundary, recording the interpolated intersection
// point at each crossing.
func (i *Impl) Slice(height float64) Polygons {
	verts := i.Verts()
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	numTri := len(starts) / 3
	if numTri == 0 {
		return nil
	}

	// Stand in for the persistent C++ collider_: rebuild it from the
	// per-face boxes/Morton codes. The faces are already Morton-sorted
	// (the SortGeometry invariant that makes collider_ valid), so the
	// Collider's leaf index equals the face index. See the PORT_NOTES
	// note on ephemerally rebuilt Colliders.
	faceBox, faceMorton := i.GetFaceBoxMorton()
	c := collider.New(faceBox, faceMorton)

	// Single query: the bBox flattened to the slice plane.
	minB, maxB := i.BBox()
	plane := geom.Box{
		Min: geom.Vec3{X: minB.X, Y: minB.Y, Z: height},
		Max: geom.Vec3{X: maxB.X, Y: maxB.Y, Z: height},
	}

	// Recorder: a candidate tri is accepted iff its z-range
	// (min ≤ height < max) actually straddles the slice plane. C++ uses
	// a set; we mirror with a Go map. The query runs serially
	// (parallel=false in C++ Slice), so no lock is needed.
	tris := make(map[int]struct{})
	c.CollisionsBox([]geom.Box{plane}, false, false, func(_, tri int) {
		minZ := verts[starts[3*tri]].Z
		maxZ := minZ
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
	})

	var polys Polygons
	for len(tris) > 0 {
		var startTri int
		for tri := range tris {
			startTri = tri
			break
		}
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
