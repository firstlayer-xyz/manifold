package boolean

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// RayHit is the Go port of the C++ RayHit struct: one ray/triangle hit.
type RayHit struct {
	FaceID   uint64
	Distance float64
	Position geom.Vec3
	Normal   geom.Vec3
}

// axis returns the i-th component of v (0=x, 1=y, 2=z).
func axis(v geom.Vec3, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// RayCast is the Go port of Manifold::Impl::RayCast (boolean3.cpp:576): every
// triangle hit of the segment origin->endpoint, sorted by distance. col is the
// target mesh's face collider (collider.New(GetFaceBoxMorton())); its leaf
// indices are the original face indices. vertPos/vertNormal/faceNormal +
// starts/pairs are the target mesh's arrays.
//
// It builds a minimal single-edge ray Impl (edge 0 forward, edge 1 reverse,
// edge 2 degenerate; zero normals, so the ray contributes nothing to the
// perturbation tiebreakers), instantiates Kernel02/11/12<false,true> over it,
// and processes each candidate triangle from the BVH query.
func RayCast(vertPos, vertNormal, faceNormal []geom.Vec3, starts, pairs []int32, col *collider.Collider, origin, endpoint geom.Vec3) []RayHit {
	dir := endpoint.Sub(origin)
	if dir.Dot(dir) == 0.0 {
		return nil
	}

	rayMesh := &mesh{
		vertPos:    []geom.Vec3{origin, endpoint},
		vertNormal: []geom.Vec3{{}, {}},
		faceNormal: []geom.Vec3{{}},
		halfedge:   halfedges{starts: []int32{0, 1, -1}, pairs: []int32{1, 0, -1}},
	}
	target := &mesh{
		vertPos:    vertPos,
		vertNormal: vertNormal,
		faceNormal: faceNormal,
		halfedge:   halfedges{starts: starts, pairs: pairs},
	}
	k12 := newKernel12(rayMesh, target, false, true)

	// Largest-magnitude component for a stable t.
	absDir := dir.Abs()
	tAxis := 2
	if absDir.X > absDir.Y && absDir.X > absDir.Z {
		tAxis = 0
	} else if absDir.Y > absDir.Z {
		tAxis = 1
	}

	var hits []RayHit
	rayBox := geom.NewBox(origin, endpoint)
	col.CollisionsBox([]geom.Box{rayBox}, false, false, func(queryIdx, tri int) {
		s, v := k12.call(0, tri) // halfedge 0 (forward ray) vs triangle tri
		if s != 0 && isFinite(v.X) {
			t := (axis(v, tAxis) - axis(origin, tAxis)) / axis(dir, tAxis)
			if t >= 0.0 && t <= 1.0 {
				hits = append(hits, RayHit{
					FaceID:   uint64(tri),
					Distance: t,
					Position: v,
					Normal:   faceNormal[tri],
				})
			}
		}
	})

	// C++ std::sort by distance (ties unspecified).
	sort.Slice(hits, func(i, j int) bool { return hits[i].Distance < hits[j].Distance })
	return hits
}
