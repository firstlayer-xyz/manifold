package manifold

import (
	"sync"

	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
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

	// Build a Collider over the impl's per-face AABBs. Although the
	// impl was already morton-sorted by SortGeometry, the bBox may
	// have been refreshed since (BuildCollider updates it), so
	// recomputing morton codes may not produce an exactly-ascending
	// sequence. We stable-sort here to guarantee Collider's
	// precondition and remember the face-id mapping so the recorder
	// can recover the original tri index.
	faceBox, faceMorton, faceID := perFaceBoxMorton(i)
	if len(faceBox) == 0 {
		return nil
	}
	policy := parallel.AutoPolicy(len(faceBox), 100000)
	order := make([]int32, len(faceBox))
	parallel.Sequence(policy, order)
	parallel.StableSort(parallel.AutoPolicy(len(faceBox), 10000), order, func(a, b int32) bool {
		return faceMorton[a] < faceMorton[b]
	})
	sortedBox := make([]geom.Box, len(faceBox))
	sortedCode := make([]uint32, len(faceBox))
	sortedFaceID := make([]int32, len(faceBox))
	parallel.ForEachN(policy, len(faceBox), func(newIdx int) {
		oldIdx := order[newIdx]
		sortedBox[newIdx] = faceBox[oldIdx]
		sortedCode[newIdx] = faceMorton[oldIdx]
		sortedFaceID[newIdx] = faceID[oldIdx]
	})
	c := collider.New(sortedBox, sortedCode)

	// Single query: the bBox flattened to the slice plane.
	minB, maxB := i.BBox()
	plane := geom.Box{
		Min: geom.Vec3{X: minB.X, Y: minB.Y, Z: height},
		Max: geom.Vec3{X: maxB.X, Y: maxB.Y, Z: height},
	}

	// Recorder: a candidate tri is accepted iff its z-range
	// (min ≤ height < max) actually straddles the slice plane.
	// C++ uses a set; we mirror with a Go map. The recorder is
	// called serially (parallel=false in C++ Slice), so no lock
	// needed — but we pass parallel_=false here for safety.
	tris := make(map[int]struct{})
	var mu sync.Mutex
	c.CollisionsBox([]geom.Box{plane}, false, false, func(_, leafIdx int) {
		tri := int(sortedFaceID[leafIdx])
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
			mu.Lock()
			tris[tri] = struct{}{}
			mu.Unlock()
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

// perFaceBoxMorton recomputes the per-face AABB + Morton code arrays
// for use by Slice's Collider, along with each face's original tri
// index. Mirrors what GetFaceBoxMorton does on MutableImpl but reads
// from a read-only Impl and skips removed-tri sentinels (Pair < 0)
// rather than emitting NoMortonCode placeholders. (We don't expose
// a const-Impl GetFaceBoxMorton because no other Go caller needs it
// yet; revisit when porting the Boolean engine.)
func perFaceBoxMorton(i *Impl) ([]geom.Box, []uint32, []int32) {
	verts := i.Verts()
	starts := i.HalfedgeStarts()
	pairs := i.HalfedgePairs()
	minB, maxB := i.BBox()
	bBox := geom.Box{Min: minB, Max: maxB}
	numTri := len(starts) / 3
	boxes := make([]geom.Box, 0, numTri)
	codes := make([]uint32, 0, numTri)
	faceIDs := make([]int32, 0, numTri)
	for face := 0; face < numTri; face++ {
		if pairs[3*face] < 0 {
			continue
		}
		p0 := verts[starts[3*face+0]]
		p1 := verts[starts[3*face+1]]
		p2 := verts[starts[3*face+2]]
		box := geom.Box{Min: p0, Max: p0}.UnionPoint(p1).UnionPoint(p2)
		center := p0.Add(p1).Add(p2).Scale(1.0 / 3.0)
		boxes = append(boxes, box)
		codes = append(codes, geom.MortonCode(center, bBox))
		faceIDs = append(faceIDs, int32(face))
	}
	return boxes, codes, faceIDs
}
