package manifold

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
)

// implStorage owns everything C++ Manifold::Impl owns (src/impl.h:56-89).
// manifold.Impl / MutableImpl are views over one implStorage; their getters/setters
// route through it, and Manifold publishes it directly (no cgo). The collider is
// held on the view/Manifold structs, not here. Marshalling implStorage to/from a C++
// bridge handle is test-only (the differential oracle seam in oracle_test.go).
type implStorage struct {
	vertPos         []geom.Vec3
	halfedge        mesh.Halfedges
	properties      []float64
	vertNormal      []geom.Vec3
	faceNormal      []geom.Vec3
	halfedgeTangent []geom.Vec4
	meshRelation    mesh.MeshRelationD
	numProp         int
	bBox            geom.Box
	epsilon         float64
	tolerance       float64
	status          Error
}

// newImplStorage returns an empty storage with the C++ default scalars (bBox =
// empty, epsilon/tolerance = -1, originalID = -1).
func newImplStorage() *implStorage {
	return &implStorage{
		meshRelation: mesh.NewMeshRelationD(),
		bBox:         geom.EmptyBox(),
		epsilon:      -1,
		tolerance:    -1,
	}
}

// clone returns a deep copy of the storage: every slice and the meshRelation
// (triRef slice + meshIDtransform map) are copied so mutations on the clone never
// reach the original. Mirrors the value semantics of the C++ Impl copy-constructor
// for the storage fields; collider_ is cloned separately by the caller (Impl.Copy).
func (s *implStorage) clone() *implStorage {
	starts, paired, propVert := s.halfedge.Raw()
	return &implStorage{
		vertPos: append([]geom.Vec3(nil), s.vertPos...),
		halfedge: mesh.NewHalfedges(
			append([]int32(nil), starts...),
			append([]int32(nil), paired...),
			append([]int32(nil), propVert...),
		),
		properties:      append([]float64(nil), s.properties...),
		vertNormal:      append([]geom.Vec3(nil), s.vertNormal...),
		faceNormal:      append([]geom.Vec3(nil), s.faceNormal...),
		halfedgeTangent: append([]geom.Vec4(nil), s.halfedgeTangent...),
		meshRelation:    cloneMeshRelation(s.meshRelation),
		numProp:         s.numProp,
		bBox:            s.bBox,
		epsilon:         s.epsilon,
		tolerance:       s.tolerance,
		status:          s.status,
	}
}

// cloneMeshRelation deep-copies a MeshRelationD (triRef slice + the ordered
// meshIDtransform map, preserving key order).
func cloneMeshRelation(mr mesh.MeshRelationD) mesh.MeshRelationD {
	out := mesh.NewMeshRelationD()
	out.OriginalID = mr.OriginalID
	out.TriRef = append([]mesh.TriRef(nil), mr.TriRef...)
	for k, r := range mr.MeshIDTransform.All() {
		out.MeshIDTransform.Set(k, r)
	}
	return out
}

// resizeVec3 grows/shrinks a []geom.Vec3 to length n (new slots zero-valued),
// reusing the backing array on shrink — the Vec semantics MutableImpl's
// Resize{Verts,VertNormals,FaceNormals} accessors need.
func resizeVec3(s []geom.Vec3, n int) []geom.Vec3 {
	if n <= len(s) {
		return s[:n]
	}
	return append(s, make([]geom.Vec3, n-len(s))...)
}

// meshIDTransformsNative returns the meshIDtransform entries in ascending meshID
// order (the native map is insertion-ordered, but C++ iterates the std::map by
// key, so callers must see key order).
func meshIDTransformsNative(mr *mesh.MeshRelationD) []meshIDRelation {
	keys := append([]int(nil), mr.MeshIDTransform.Keys()...)
	sort.Ints(keys)
	out := make([]meshIDRelation, 0, len(keys))
	for _, k := range keys {
		r, _ := mr.MeshIDTransform.Get(k)
		out = append(out, meshIDRelation{
			MeshID: k, OriginalID: r.OriginalID,
			Transform: r.Transform, BackSide: r.BackSide, HasNormals: r.HasNormals,
		})
	}
	return out
}

// tangentsFromFlat unpacks the bridge's flat 4-per-element tangent buffer into
// Vec4s; tangentsToFlat is the inverse.
func tangentsFromFlat(flat []float64) []geom.Vec4 {
	out := make([]geom.Vec4, len(flat)/4)
	for i := range out {
		out[i] = geom.Vec4{X: flat[4*i], Y: flat[4*i+1], Z: flat[4*i+2], W: flat[4*i+3]}
	}
	return out
}

func tangentsToFlat(tangents []geom.Vec4) []float64 {
	flat := make([]float64, 4*len(tangents))
	for i, t := range tangents {
		flat[4*i], flat[4*i+1], flat[4*i+2], flat[4*i+3] = t.X, t.Y, t.Z, t.W
	}
	return flat
}
