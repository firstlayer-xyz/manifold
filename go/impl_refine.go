// Refine (the smoothing path of subdivision) ports src/smoothing.cpp's
// Manifold::Impl::Refine and its InterpTri functor — the cubic-Bezier surface
// interpolation that moves the new subdivision verts onto the smooth surface
// implied by the halfedge tangents. Topology comes from Subdivide
// (impl_subdivide.go); this file is the geometry.
package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/mesh"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// orthogonalTo is the Go port of OrthogonalTo (smoothing.cpp:27): the component of
// `in` orthogonal to `ref`, normalized; falls back to altIn when in is ~parallel
// to ref.
func orthogonalTo(in, altIn, ref Vec3) Vec3 {
	out := in.Sub(ref.Scale(in.Dot(ref)))
	if out.Dot(out) < kPrecision*in.Dot(in) {
		out = altIn.Sub(ref.Scale(altIn.Dot(ref)))
	}
	return out.SafeNormalize()
}

func mat3Cols(c0, c1, c2 Vec3) geom.Mat3 {
	return geom.Mat3{{c0.X, c0.Y, c0.Z}, {c1.X, c1.Y, c1.Z}, {c2.X, c2.Y, c2.Z}}
}

// --- InterpTri static helpers (smoothing.cpp:78-183) ---

func homogeneous4(v geom.Vec4) geom.Vec4 {
	return geom.Vec4{X: v.X * v.W, Y: v.Y * v.W, Z: v.Z * v.W, W: v.W}
}

func homogeneous3(v Vec3) geom.Vec4 { return geom.Vec4{X: v.X, Y: v.Y, Z: v.Z, W: 1} }

func hNormalize(v geom.Vec4) Vec3 {
	if v.W == 0 {
		return v.Vec3()
	}
	return v.Vec3().Scale(1 / v.W)
}

func bezier(point Vec3, tangent geom.Vec4) geom.Vec4 {
	return homogeneous4(geom.Vec4{X: point.X + tangent.X, Y: point.Y + tangent.Y, Z: point.Z + tangent.Z, W: tangent.W})
}

func cubicBezier2Linear(p0, p1, p2, p3 geom.Vec4, x float64) [2]geom.Vec4 {
	p12 := p1.Lerp(p2, x)
	return [2]geom.Vec4{
		p0.Lerp(p1, x).Lerp(p12, x),
		p12.Lerp(p2.Lerp(p3, x), x),
	}
}

func bezierPoint(points [2]geom.Vec4, x float64) Vec3 {
	return hNormalize(points[0].Lerp(points[1], x))
}

func bezierTangent(points [2]geom.Vec4) Vec3 {
	return hNormalize(points[1]).Sub(hNormalize(points[0])).SafeNormalize()
}

func rotateFromTo(v Vec3, start, end geom.Vec4) Vec3 {
	return geom.Qrot(end, geom.Qrot(geom.Qconj(start), v))
}

// slerp is the Go port of InterpTri::Slerp (smoothing.cpp:118).
func slerp(x, y geom.Vec4, a float64, longWay bool) geom.Vec4 {
	z := y
	cosTheta := x.Dot(y)
	if (cosTheta < 0) != longWay {
		z = y.Scale(-1)
		cosTheta = -cosTheta
	}
	if math.Abs(cosTheta) > 1.0-float64EpsilonRefine {
		return x.Lerp(z, a)
	}
	angle := math.Acos(cosTheta)
	return x.Scale(math.Sin((1-a)*angle)).Add(z.Scale(math.Sin(a * angle))).Scale(1 / math.Sin(angle))
}

const float64EpsilonRefine = 2.220446049250313e-16

func bezier2Bezier(corners [2]Vec3, tangentsX, tangentsY [2]geom.Vec4, x float64, anchor Vec3) [2]geom.Vec4 {
	bez := cubicBezier2Linear(
		homogeneous3(corners[0]), bezier(corners[0], tangentsX[0]),
		bezier(corners[1], tangentsX[1]), homogeneous3(corners[1]), x)
	end := bezierPoint(bez, x)
	tangent := bezierTangent(bez)

	nTangentsX := [2]Vec3{
		tangentsX[0].Vec3().SafeNormalize(),
		tangentsX[1].Vec3().SafeNormalize().Scale(-1),
	}
	biTangents := [2]Vec3{
		orthogonalTo(tangentsY[0].Vec3(), anchor.Sub(corners[0]), nTangentsX[0]),
		orthogonalTo(tangentsY[1].Vec3(), anchor.Sub(corners[1]), nTangentsX[1]),
	}
	q0 := geom.RotationQuatFromMat3(mat3Cols(nTangentsX[0], biTangents[0], nTangentsX[0].Cross(biTangents[0])))
	q1 := geom.RotationQuatFromMat3(mat3Cols(nTangentsX[1], biTangents[1], nTangentsX[1].Cross(biTangents[1])))
	edge := corners[1].Sub(corners[0])
	longWay := nTangentsX[0].Dot(edge)+nTangentsX[1].Dot(edge) < 0
	qTmp := slerp(q0, q1, x, longWay)
	q := geom.Qmul(geom.RotationQuatFromTo(geom.Qxdir(qTmp), tangent), qTmp)

	delta := lerpVec3(rotateFromTo(tangentsY[0].Vec3(), q0, q), rotateFromTo(tangentsY[1].Vec3(), q1, q), x)
	deltaW := lerpScalar(tangentsY[0].W, tangentsY[1].W, x)
	return [2]geom.Vec4{homogeneous3(end), {X: delta.X, Y: delta.Y, Z: delta.Z, W: deltaW}}
}

func bezier2D(corners [4]Vec3, tangentsX, tangentsY [4]geom.Vec4, x, y float64, centroid Vec3) Vec3 {
	bez0 := bezier2Bezier([2]Vec3{corners[0], corners[1]}, [2]geom.Vec4{tangentsX[0], tangentsX[1]},
		[2]geom.Vec4{tangentsY[0], tangentsY[1]}, x, centroid)
	bez1 := bezier2Bezier([2]Vec3{corners[2], corners[3]}, [2]geom.Vec4{tangentsX[2], tangentsX[3]},
		[2]geom.Vec4{tangentsY[2], tangentsY[3]}, 1-x, centroid)
	bez := cubicBezier2Linear(bez0[0], bezier(bez0[0].Vec3(), bez0[1]),
		bezier(bez1[0].Vec3(), bez1[1]), bez1[0], y)
	return bezierPoint(bez, y)
}

// interpTri is the Go port of InterpTri::operator() (smoothing.cpp:185): place the
// new subdivision vert `vert` on the smooth Bezier surface of its originating
// triangle/quad in `old`. Writes vertPos[vert].
func interpTri(vertPos []Vec3, vertBary []mesh.Barycentric, old *MutableImpl, vert int) {
	tri := vertBary[vert].Tri
	uvw := vertBary[vert].UVW
	halfedges := old.getHalfedges(tri)

	var corners [4]Vec3
	for i := 0; i < 3; i++ {
		corners[i] = old.s.vertPos[old.s.halfedge.Start(halfedges[i])]
	}
	if halfedges[3] >= 0 {
		corners[3] = old.s.vertPos[old.s.halfedge.Start(halfedges[3])]
	}

	for i := 0; i < 4; i++ {
		if getVec4Comp(uvw, i) == 1 {
			vertPos[vert] = corners[i]
			return
		}
	}

	var posH geom.Vec4
	tangents := old.s.halfedgeTangent
	if halfedges[3] < 0 { // tri
		tangentR := [3]geom.Vec4{tangents[halfedges[0]], tangents[halfedges[1]], tangents[halfedges[2]]}
		tangentL := [3]geom.Vec4{
			tangents[old.s.halfedge.Pair(halfedges[2])],
			tangents[old.s.halfedge.Pair(halfedges[0])],
			tangents[old.s.halfedge.Pair(halfedges[1])],
		}
		centroid := corners[0].Add(corners[1]).Add(corners[2]).Scale(1.0 / 3)
		for i := 0; i < 3; i++ {
			j := geom.Next3(i)
			k := geom.Prev3(i)
			x := getVec4Comp(uvw, k) / (1 - getVec4Comp(uvw, i))
			bez := bezier2Bezier([2]Vec3{corners[j], corners[k]}, [2]geom.Vec4{tangentR[j], tangentL[k]},
				[2]geom.Vec4{tangentL[j], tangentR[k]}, x, centroid)
			bez1 := cubicBezier2Linear(bez[0], bezier(bez[0].Vec3(), bez[1]),
				bezier(corners[i], tangentR[i].Lerp(tangentL[i], x)), homogeneous3(corners[i]), getVec4Comp(uvw, i))
			p := bezierPoint(bez1, getVec4Comp(uvw, i))
			posH = posH.Add(homogeneous4(geom.Vec4{X: p.X, Y: p.Y, Z: p.Z, W: getVec4Comp(uvw, j) * getVec4Comp(uvw, k)}))
		}
	} else { // quad
		tangentsX := [4]geom.Vec4{
			tangents[halfedges[0]],
			tangents[old.s.halfedge.Pair(halfedges[0])],
			tangents[halfedges[2]],
			tangents[old.s.halfedge.Pair(halfedges[2])],
		}
		tangentsY := [4]geom.Vec4{
			tangents[old.s.halfedge.Pair(halfedges[3])],
			tangents[halfedges[1]],
			tangents[old.s.halfedge.Pair(halfedges[1])],
			tangents[halfedges[3]],
		}
		centroid := corners[0].Add(corners[1]).Add(corners[2]).Add(corners[3]).Scale(0.25)
		x := getVec4Comp(uvw, 1) + getVec4Comp(uvw, 2)
		y := getVec4Comp(uvw, 2) + getVec4Comp(uvw, 3)
		pX := bezier2D(corners, tangentsX, tangentsY, x, y, centroid)
		pY := bezier2D([4]Vec3{corners[1], corners[2], corners[3], corners[0]},
			[4]geom.Vec4{tangentsY[1], tangentsY[2], tangentsY[3], tangentsY[0]},
			[4]geom.Vec4{tangentsX[1], tangentsX[2], tangentsX[3], tangentsX[0]}, y, 1-x, centroid)
		posH = posH.Add(homogeneous4(geom.Vec4{X: pX.X, Y: pX.Y, Z: pX.Z, W: x * (1 - x)}))
		posH = posH.Add(homogeneous4(geom.Vec4{X: pY.X, Y: pY.Y, Z: pY.Z, W: y * (1 - y)}))
	}
	pos := hNormalize(posH)
	if !pos.IsFinite() {
		pos = corners[0]
	}
	vertPos[vert] = pos
}

// validTangents is the Go port of Impl::ValidTangents (smoothing.cpp:1067): a
// halfedge and its pair must agree on inside-quad status, and missing tangents
// (quad interiors) cannot be adjacent.
func (mi *MutableImpl) validTangents() bool {
	numHalfedge := mi.s.halfedge.Size()
	return parallel.AllOf(parallel.AutoPolicy(numHalfedge, 10000), numHalfedge, func(edgeIdx int) bool {
		inQuad := mi.isMarkedInsideQuad(edgeIdx)
		pair := mi.s.halfedge.Pair(edgeIdx)
		if inQuad != mi.isMarkedInsideQuad(pair) {
			return false
		}
		if !inQuad {
			return true
		}
		if mi.isMarkedInsideQuad(nextHalfedge(edgeIdx)) ||
			mi.isMarkedInsideQuad(prevHalfedge(edgeIdx)) ||
			mi.isMarkedInsideQuad(nextHalfedge(pair)) ||
			mi.isMarkedInsideQuad(prevHalfedge(pair)) {
			return false
		}
		return true
	})
}

// Refine is the Go port of Manifold::Impl::Refine (smoothing.cpp:1085): Subdivide,
// then (if the input has tangents) move the new verts onto the smooth Bezier
// surface via InterpTri, then recompute normals + finalize.
func (mi *MutableImpl) Refine(edgeDivisions func(Vec3, geom.Vec4, geom.Vec4) int, keepInterior bool) {
	if mi.NumTri() == 0 { // IsEmpty
		return
	}
	if !mi.validTangents() {
		mi.MakeEmpty(int(InvalidTangents))
		return
	}

	// Snapshot the pre-subdivide geometry InterpTri reads (verts, halfedge,
	// tangents) — C++ keeps a full copy `old`; InterpTri only reads these.
	oldHasTangents := len(mi.s.halfedgeTangent) == mi.s.halfedge.Size()
	old := &MutableImpl{s: newImplStorage()}
	old.s.vertPos = append([]Vec3(nil), mi.s.vertPos...)
	os, op, opv := mi.s.halfedge.Raw()
	old.s.halfedge = mesh.NewHalfedges(
		append([]int32(nil), os...), append([]int32(nil), op...), append([]int32(nil), opv...))
	old.s.halfedgeTangent = append([]geom.Vec4(nil), mi.s.halfedgeTangent...)

	vertBary := mi.Subdivide(edgeDivisions, keepInterior)
	if len(vertBary) == 0 {
		return
	}

	if oldHasTangents {
		verts := mi.s.vertPos
		parallel.ForEachN(parallel.AutoPolicy(mi.NumTri(), 10000), mi.NumVert(), func(vert int) {
			interpTri(verts, vertBary, old, vert)
		})
	}

	mi.s.halfedgeTangent = nil // halfedgeTangent_.clear()
	if oldHasTangents {
		mi.SetNormalsAndCoplanar()
		mi.CalculateBBox()
	} else {
		mi.CalculateVertNormals()
	}
	mi.SortGeometry()
	mi.s.meshRelation.OriginalID = -1
	mi.coll = nil
}
