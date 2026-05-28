package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// shape identifies one of the primitive meshes that
// Manifold::Impl(Shape, mat3x4) knows how to construct. Matches the
// integer values of the C++ enum class Manifold::Impl::Shape.
type shape int

const (
	shapeTetrahedron shape = 0
	shapeCube        shape = 1
	shapeOctahedron  shape = 2
)

// shapeTables holds the canonical vertex positions and triangle index
// lists for each primitive shape, copied verbatim from C++
// Manifold::Impl::Impl(Shape, mat3x4) (src/impl.cpp).
//
// vertPos values are pre-transform; the constructor applies the
// caller-supplied mat3x4 to each one. triVerts is a flat list of
// (i0, i1, i2) triples, one per triangle.
var shapeTables = map[shape]struct {
	verts    []geom.Vec3
	triVerts []int32
}{
	shapeTetrahedron: {
		verts: []geom.Vec3{
			{X: -1, Y: -1, Z: 1},
			{X: -1, Y: 1, Z: -1},
			{X: 1, Y: -1, Z: -1},
			{X: 1, Y: 1, Z: 1},
		},
		triVerts: []int32{
			2, 0, 1,
			0, 3, 1,
			2, 3, 0,
			3, 2, 1,
		},
	},
	shapeCube: {
		verts: []geom.Vec3{
			{X: 0, Y: 0, Z: 0},
			{X: 0, Y: 0, Z: 1},
			{X: 0, Y: 1, Z: 0},
			{X: 0, Y: 1, Z: 1},
			{X: 1, Y: 0, Z: 0},
			{X: 1, Y: 0, Z: 1},
			{X: 1, Y: 1, Z: 0},
			{X: 1, Y: 1, Z: 1},
		},
		triVerts: []int32{
			1, 0, 4, 2, 4, 0,
			1, 3, 0, 3, 1, 5,
			3, 2, 0, 3, 7, 2,
			5, 4, 6, 5, 1, 4,
			6, 4, 2, 7, 6, 2,
			7, 3, 5, 7, 5, 6,
		},
	},
	shapeOctahedron: {
		verts: []geom.Vec3{
			{X: 1, Y: 0, Z: 0},
			{X: -1, Y: 0, Z: 0},
			{X: 0, Y: 1, Z: 0},
			{X: 0, Y: -1, Z: 0},
			{X: 0, Y: 0, Z: 1},
			{X: 0, Y: 0, Z: -1},
		},
		triVerts: []int32{
			0, 2, 4, 1, 5, 3,
			2, 1, 4, 3, 5, 0,
			1, 3, 4, 0, 5, 2,
			3, 0, 4, 2, 5, 1,
		},
	},
}

// newImplFromShape is the Go-orchestrated port of the C++
// Manifold::Impl(Shape, mat3x4) constructor (src/impl.cpp). The shape
// table + transform happens in Go; downstream initialization steps
// (CreateHalfedges, InitializeOriginal, CalculateBBox, SetEpsilon,
// SortGeometry, SetNormalsAndCoplanar) still call into C++ via the
// existing bridge mutators.
//
// Returns the MutableImpl; the caller is responsible for converting
// it to a Manifold (via ToManifold) and for Delete'ing it. Returning
// the mutable form lets callers (notably Sphere) continue mutating
// before sealing the result.
func newImplFromShape(sh shape, m geom.Mat3x4) *bridge.MutableImpl {
	table := shapeTables[sh]
	mi := bridge.NewMutableImpl()

	// vertPos_ = Vec(vertPos); then transform each in place.
	mi.ResizeVerts(len(table.verts))
	verts := mi.Verts()
	for i, v := range table.verts {
		verts[i] = m.ApplyAffine(v)
	}

	// CreateHalfedges(triVerts) — drilled to Go (impl_halfedges.go).
	createHalfedges(mi, table.triVerts)
	// InitializeOriginal()
	mi.InitializeOriginal()
	// CalculateBBox()
	mi.CalculateBBox()
	// SetEpsilon()
	mi.SetEpsilon()
	// SortGeometry()
	mi.SortGeometry()
	// SetNormalsAndCoplanar()
	mi.SetNormalsAndCoplanar()

	return mi
}
