package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// tangentState is the read-side snapshot of the Impl arrays the tangent
// generation reads, mirroring the const Impl methods GetNormal /
// TangentFromNormal / IsInsideQuad / ForVert (src/smoothing.cpp, src/impl.h).
// It holds the data CreateTangents consults; the computed tangents are written
// back via the MutableImpl by the caller (increments B/C).
type tangentState struct {
	verts       []geom.Vec3
	starts      []int32
	pairs       []int32
	props       []int32
	numProp     int
	properties  []float64
	triRefs     []bridge.TriRef
	faceNormals []geom.Vec3
	// halfedgeTangent is the existing member tangent array (flat, 4 floats per
	// halfedge). Empty while CreateTangents is still computing tangents, in
	// which case IsInsideQuad falls back to the triRef test.
	halfedgeTangent []float64
	// meshIDtransform mirrors meshRelation_.meshIDtransform (a std::map keyed
	// by meshID); built once from the relation list for GetNormal lookups.
	meshIDtransform map[int32]bridge.MeshIDRelation
}

func newTangentState(mi *MutableImpl) *tangentState {
	m := make(map[int32]bridge.MeshIDRelation)
	for _, rel := range mi.MeshIDTransforms() {
		m[rel.MeshID] = rel
	}
	return &tangentState{
		verts:           append([]geom.Vec3(nil), mi.Verts()...),
		starts:          append([]int32(nil), mi.HalfedgeStarts()...),
		pairs:           append([]int32(nil), mi.HalfedgePairs()...),
		props:           append([]int32(nil), mi.HalfedgeProps()...),
		numProp:         mi.NumProp(),
		properties:      append([]float64(nil), mi.Properties()...),
		triRefs:         append([]bridge.TriRef(nil), mi.TriRefs()...),
		faceNormals:     append([]geom.Vec3(nil), mi.FaceNormals()...),
		halfedgeTangent: append([]float64(nil), mi.HalfedgeTangents()...),
		meshIDtransform: m,
	}
}

func (ts *tangentState) start(he int) int { return int(ts.starts[he]) }

// end mirrors Halfedges::End: the start vert of the next halfedge in the tri.
func (ts *tangentState) end(he int) int { return int(ts.starts[nextHalfedge(he)]) }

// circularTangent is the Go port of CircularTangent (src/smoothing.cpp:61):
// a circular (quadratic-weighted Bezier) tangent for an edge, returned in
// geometric (de-homogenized) form. Float-expression order matches C++ exactly
// (left-associative scalar multiplies; component-wise divide, not reciprocal).
func circularTangent(tangent, edgeVec geom.Vec3) geom.Vec4 {
	dir := tangent.SafeNormalize()

	// std::max(0.5, dot) — returns 0.5 unless dot is strictly greater (and
	// keeps 0.5 on NaN, unlike math.Max).
	weight := 0.5
	if d := dir.Dot(edgeVec.SafeNormalize()); 0.5 < d {
		weight = d
	}
	// Quadratic weighted bezier for circular interpolation.
	v := dir.Scale(0.5).Scale(edgeVec.Length()) // dir * 0.5 * length(edgeVec)
	bz2 := geom.Vec4{X: v.X, Y: v.Y, Z: v.Z, W: weight}
	// Equivalent cubic weighted bezier.
	bz3 := geom.Vec4{X: 0, Y: 0, Z: 0, W: 1}.Lerp(bz2, 2.0/3.0)
	// Convert from homogeneous form to geometric form: vec3(bz3) / bz3.w.
	return geom.Vec4{X: bz3.X / bz3.W, Y: bz3.Y / bz3.W, Z: bz3.Z / bz3.W, W: bz3.W}
}

// getNormal is the Go port of Impl::GetNormal (src/smoothing.cpp:270): read the
// vertex normal from property slot normalIdx of the halfedge's propVert. When
// the source mesh's relation is not flagged hasNormals, the stored slot is in
// the per-mesh frame and is re-rotated to world via the relation's normal
// transform (NormalTransform(transform) * (backSide ? -1 : 1)).
func (ts *tangentState) getNormal(halfedge, normalIdx int) geom.Vec3 {
	prop := int(ts.props[halfedge])
	normal := geom.Vec3{
		X: ts.properties[prop*ts.numProp+normalIdx+0],
		Y: ts.properties[prop*ts.numProp+normalIdx+1],
		Z: ts.properties[prop*ts.numProp+normalIdx+2],
	}
	meshID := ts.triRefs[halfedge/3].MeshID
	if rel, ok := ts.meshIDtransform[meshID]; ok && !rel.HasNormals {
		s := 1.0
		if rel.BackSide {
			s = -1.0
		}
		nt := geom.NormalTransform(geom.Mat3x4(rel.Transform)).MulScalar(s)
		normal = nt.MulVec3(normal)
	}
	return normal
}

// tangentFromNormal is the Go port of Impl::TangentFromNormal
// (src/smoothing.cpp:293): a circular tangent for the halfedge, orthogonal to
// the given normal, choosing the bitangent sense to avoid folding past 90°.
func (ts *tangentState) tangentFromNormal(normal geom.Vec3, halfedge int) geom.Vec4 {
	edgeVec := ts.verts[ts.end(halfedge)].Sub(ts.verts[ts.start(halfedge)])
	edgeNormal := ts.faceNormals[halfedge/3].Add(ts.faceNormals[int(ts.pairs[halfedge])/3])
	var biTangent geom.Vec3
	if normal.Dot(edgeNormal) < 0 {
		biTangent = edgeNormal.Cross(edgeVec)
	} else {
		biTangent = normal.Cross(edgeVec)
	}
	return circularTangent(biTangent.Cross(normal), edgeVec)
}

// isInsideQuad is the Go port of Impl::IsInsideQuad (src/smoothing.cpp:309):
// true if the halfedge is the interior diagonal of a quad — its two triangles
// refer to the same face and neither has further same-face neighbors. Once
// halfedge tangents exist, the marker is the negative-weight tangent instead.
func (ts *tangentState) isInsideQuad(halfedge int) bool {
	if len(ts.halfedgeTangent) > 0 {
		return ts.halfedgeTangent[4*halfedge+3] < 0
	}
	tri := halfedge / 3
	ref := ts.triRefs[tri]
	pair := int(ts.pairs[halfedge])
	pairTri := pair / 3
	pairRef := ts.triRefs[pairTri]
	if !triRefSameFace(ref, pairRef) {
		return false
	}
	sameFace := func(he int, ref bridge.TriRef) bool {
		return triRefSameFace(ref, ts.triRefs[int(ts.pairs[he])/3])
	}
	neighbor := nextHalfedge(halfedge)
	if sameFace(neighbor, ref) {
		return false
	}
	neighbor = nextHalfedge(neighbor)
	if sameFace(neighbor, ref) {
		return false
	}
	neighbor = nextHalfedge(pair)
	if sameFace(neighbor, pairRef) {
		return false
	}
	neighbor = nextHalfedge(neighbor)
	if sameFace(neighbor, pairRef) {
		return false
	}
	return true
}

// forVert is the Go port of Impl::ForVert (src/impl.h:253): walk the halfedges
// around the start vert of `halfedge` (via Next(Pair(current))) once, applying
// fn to each, returning when the ring closes.
func (ts *tangentState) forVert(halfedge int, fn func(int)) {
	current := halfedge
	for {
		current = nextHalfedge(int(ts.pairs[current]))
		fn(current)
		if current == halfedge {
			break
		}
	}
}

// forVertT is the Go port of the transform/binaryOp form of Impl::ForVert
// (src/impl.h:262): transform each halfedge around the vert to a T, and apply
// binaryOp to each adjacent (here, next) pair around the ring.
func forVertT[T any](ts *tangentState, halfedge int, transform func(int) T, binaryOp func(he int, here, next T)) {
	here := transform(halfedge)
	current := halfedge
	for {
		nh := nextHalfedge(int(ts.pairs[current]))
		next := transform(nh)
		binaryOp(current, here, next)
		here = next
		current = nh
		if current == halfedge {
			break
		}
	}
}
