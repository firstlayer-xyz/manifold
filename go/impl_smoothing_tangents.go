package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

const (
	kPi    = math.Pi
	kTwoPi = 2 * math.Pi
)

// tangentState is the read-side snapshot of the Impl arrays the tangent
// generation reads, mirroring the const Impl methods GetNormal /
// TangentFromNormal / IsInsideQuad / ForVert (src/smoothing.cpp, src/impl.h).
// It holds the data CreateTangents consults; the computed tangents are written
// back via the MutableImpl by the caller (increments B/C).
type tangentState struct {
	verts       []geom.Vec3
	vertNormals []geom.Vec3
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
		vertNormals:     append([]geom.Vec3(nil), mi.VertNormals()...),
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

// ForVert iteration uses the shared forVert / forVertTransform helpers in
// impl_halfedge_walk.go (passing ts.pairs) — the faithful Impl::ForVert ports.

// wrapAngle mirrors Wrap (src/smoothing.cpp:35): bring radians into (-π, π].
func wrapAngle(radians float64) float64 {
	if radians < -kPi {
		return radians + kTwoPi
	}
	if radians > kPi {
		return radians - kTwoPi
	}
	return radians
}

// equalNormals mirrors EqualNormals (src/smoothing.cpp:52).
func equalNormals(a, b geom.Vec3) bool {
	return a.SafeNormalize().Dot(b.SafeNormalize()) > 0.9999
}

// isMarkedInsideQuad mirrors Impl::IsMarkedInsideQuad (src/smoothing.cpp:336):
// a quad interior is marked by a negative-weight halfedge tangent.
func isMarkedInsideQuad(tangent []geom.Vec4, halfedge int) bool {
	return len(tangent) > 0 && tangent[halfedge].W < 0
}

// vertHalfedge is the Go port of Impl::VertHalfedge (src/smoothing.cpp:409):
// one arbitrary halfedge starting at each vert. C++ races via atomic_exchange
// ("arbitrary winner"); the result is start-invariant for the consumers, so
// the sequential lowest-index choice is equivalent.
func (ts *tangentState) vertHalfedge() []int {
	numVert := len(ts.verts)
	vh := make([]int, numVert)
	counters := make([]uint8, numVert)
	for idx := 0; idx < len(ts.starts); idx++ {
		start := int(ts.starts[idx])
		if counters[start] == 1 {
			continue
		}
		counters[start] = 1
		vh[start] = idx
	}
	return vh
}

// distributeTangents is the Go port of Impl::DistributeTangents
// (src/smoothing.cpp:689): even out the angular spacing of the tangents around
// each vert between fixed tangents, rotating each free tangent about the vert
// normal so the current angular layout matches the desired (edge-proportional)
// one. Operates in place on the working `tangent` array.
func (ts *tangentState) distributeTangents(tangent []geom.Vec4, fixedHalfedges []bool) {
	process := func(halfedge int) {
		if !fixedHalfedges[halfedge] || isMarkedInsideQuad(tangent, halfedge) {
			return
		}
		normal := geom.Vec3{}
		var currentAngle, desiredAngle []float64

		approxNormal := ts.vertNormals[ts.start(halfedge)]
		center := ts.verts[ts.start(halfedge)]
		lastEdgeVec := ts.verts[ts.end(halfedge)].Sub(center).SafeNormalize()
		firstTangent := tangent[halfedge].Vec3().SafeNormalize()
		lastTangent := firstTangent
		current := halfedge
		for {
			current = nextHalfedge(int(ts.pairs[current]))
			if !isMarkedInsideQuad(tangent, current) {
				thisEdgeVec := ts.verts[ts.end(current)].Sub(center).SafeNormalize()
				thisTangent := tangent[current].Vec3().SafeNormalize()
				normal = normal.Add(thisTangent.Cross(lastTangent))
				// cumulative sum
				da := geom.AngleBetween(thisEdgeVec, lastEdgeVec)
				if len(desiredAngle) > 0 {
					da += desiredAngle[len(desiredAngle)-1]
				}
				desiredAngle = append(desiredAngle, da)
				if current == halfedge {
					currentAngle = append(currentAngle, kTwoPi)
				} else {
					ca := geom.AngleBetween(thisTangent, firstTangent)
					if approxNormal.Dot(thisTangent.Cross(firstTangent)) < 0 {
						ca = kTwoPi - ca
					}
					currentAngle = append(currentAngle, ca)
				}
				lastEdgeVec = thisEdgeVec
				lastTangent = thisTangent
			}
			if fixedHalfedges[current] {
				break
			}
		}

		if len(currentAngle) == 1 || normal.Dot(normal) == 0 {
			return
		}

		scale := currentAngle[len(currentAngle)-1] / desiredAngle[len(desiredAngle)-1]
		offset := 0.0
		if current == halfedge { // only one - find average offset
			for i := 0; i < len(currentAngle); i++ {
				offset += wrapAngle(currentAngle[i] - scale*desiredAngle[i])
			}
			offset /= float64(len(currentAngle))
		}

		current = halfedge
		i := 0
		for {
			current = nextHalfedge(int(ts.pairs[current]))
			if current != halfedge && fixedHalfedges[current] {
				break
			}
			if !isMarkedInsideQuad(tangent, current) {
				desiredAngle[i] *= scale
				lastAngle := 0.0
				if i > 0 {
					lastAngle = desiredAngle[i-1]
				}
				// shrink obtuse angles
				if desiredAngle[i]-lastAngle > kPi {
					desiredAngle[i] = lastAngle + kPi
				} else if i+1 < len(desiredAngle) && scale*desiredAngle[i+1]-desiredAngle[i] > kPi {
					desiredAngle[i] = scale*desiredAngle[i+1] - kPi
				}
				angle := currentAngle[i] - desiredAngle[i] - offset
				t := tangent[current].Vec3()
				q := geom.RotationQuat(normal.Normalize(), angle)
				newTangent := geom.Qrot(q, t)
				tangent[current].X = newTangent.X
				tangent[current].Y = newTangent.Y
				tangent[current].Z = newTangent.Z
				i++
			}
			if fixedHalfedges[current] {
				break
			}
		}
	}

	for halfedge := 0; halfedge < len(fixedHalfedges); halfedge++ {
		process(halfedge)
	}
}

// createTangentsIdx is the Go port of Impl::CreateTangents(int)
// (src/smoothing.cpp:781): compute halfedgeTangent_ as weighted cubic Beziers
// constrained to the per-halfedge normals in property slot normalIdx, creating
// circular arcs where the two endpoint normals agree, fixing tangents at normal
// discontinuities, and aligning flat-face and missing-normal cases, then
// evening the angular distribution via DistributeTangents.
func (mi *MutableImpl) createTangentsIdx(normalIdx int) {
	ts := newTangentState(mi)
	ts.halfedgeTangent = nil // C++ halfedgeTangent_.clear() (line 785)
	numVert := len(ts.verts)
	numHalfedge := len(ts.starts)
	tangent := make([]geom.Vec4, numHalfedge)
	fixedHalfedge := make([]bool, numHalfedge)

	// special flags for tangent.w
	const kInsideQuad = -1.0
	const kMissingNormal = -3.0

	vertHalfedge := ts.vertHalfedge()
	for v := 0; v < numVert; v++ {
		e := vertHalfedge[v]

		type flatNormal struct {
			isFlatFace bool
			normal     geom.Vec3
		}
		faceEdges := [2]int{-1, -1}
		startHalfedge := -1
		lastNormal := geom.Vec3{}

		forVertTransform[flatNormal](e, ts.pairs,
			func(halfedge int) flatNormal {
				normal := ts.getNormal(halfedge, normalIdx)
				return flatNormal{
					isFlatFace: equalNormals(normal, ts.faceNormals[halfedge/3]),
					normal:     normal,
				}
			},
			func(halfedge int, here flatNormal, next *flatNormal) {
				// Tangents not known yet are temporary normal storage with a
				// negative flag w; this starts with the flag clear.
				tangent[halfedge].W = 1

				if here.isFlatFace != next.isFlatFace {
					// Record the two halfedges bordering a single flat face.
					if faceEdges[0] == -1 {
						faceEdges[0] = halfedge
					} else if faceEdges[1] == -1 {
						faceEdges[1] = halfedge
					} else {
						faceEdges[0] = -2
					}
				}

				if next.normal == (geom.Vec3{}) || here.normal == (geom.Vec3{}) {
					if here.normal != (geom.Vec3{}) { // next missing
						lastNormal = here.normal
					} else if next.normal != (geom.Vec3{}) { // here missing
						if startHalfedge < 0 {
							startHalfedge = halfedge
						}
					} else { // both missing
						if startHalfedge < 0 {
							startHalfedge = -2
						}
					}
					tangent[halfedge] = geom.Vec4{X: lastNormal.X, Y: lastNormal.Y, Z: lastNormal.Z, W: kMissingNormal}
				}

				if ts.isInsideQuad(halfedge) {
					tangent[halfedge] = geom.Vec4{X: lastNormal.X, Y: lastNormal.Y, Z: lastNormal.Z, W: kInsideQuad}
				}

				if tangent[halfedge].W < 0 {
					return
				}

				// calculate tangents
				if equalNormals(next.normal, here.normal) {
					tangent[halfedge] = ts.tangentFromNormal(here.normal, halfedge)
				} else {
					// tangents at the intersection of two normals are fixed.
					fixedHalfedge[halfedge] = true
					// Override the flat face logic if more than one normal.
					faceEdges[0] = -2

					edgeVec := ts.verts[ts.end(halfedge)].Sub(ts.verts[ts.start(halfedge)])
					dir := here.normal.Cross(next.normal)
					s := 1.0
					if dir.Dot(edgeVec) < 0 {
						s = -1.0
					}
					tangent[halfedge] = circularTangent(dir.Scale(s), edgeVec)
				}
			})

		if startHalfedge != -1 && lastNormal == (geom.Vec3{}) {
			// Use vert pseudo normal if no normals are present at all.
			normal := ts.vertNormals[ts.start(e)]
			forVert(e, ts.pairs, func(halfedge int) {
				if tangent[halfedge].W != kInsideQuad {
					tangent[halfedge] = ts.tangentFromNormal(normal, halfedge)
				}
			})
			continue
		}

		if startHalfedge >= 0 {
			// Orbit the vertex backwards, pulling the next normal from the
			// tangent where it is stored temporarily.
			current := startHalfedge
			prevNormal := ts.getNormal(nextHalfedge(int(ts.pairs[current])), normalIdx)
			for {
				if tangent[current].W == kMissingNormal {
					nextNormal := tangent[current].Vec3()
					if nextNormal == (geom.Vec3{}) {
						nextNormal = lastNormal
					}
					if equalNormals(prevNormal, nextNormal) {
						tangent[current] = ts.tangentFromNormal(prevNormal, current)
					} else {
						dir := prevNormal.Cross(nextNormal)
						edgeVec := ts.verts[ts.end(current)].Sub(ts.verts[ts.start(current)])
						s := 1.0
						if dir.Dot(edgeVec) < 0 {
							s = -1.0
						}
						tangent[current] = circularTangent(dir.Scale(s), edgeVec)
					}
				}
				currentNormal := ts.getNormal(current, normalIdx)
				if currentNormal != (geom.Vec3{}) {
					prevNormal = currentNormal
				}
				current = int(ts.pairs[prevHalfedge(current)])
				if current == startHalfedge {
					break
				}
			}
		}

		if faceEdges[0] >= 0 && faceEdges[1] >= 0 {
			// A single flat face with one shared normal: align the tangents on
			// either side to give a continuous curve.
			edge0 := ts.verts[ts.end(faceEdges[0])].Sub(ts.verts[ts.start(faceEdges[0])])
			edge1 := ts.verts[ts.end(faceEdges[1])].Sub(ts.verts[ts.start(faceEdges[1])])
			newTangent := edge0.Normalize().Sub(edge1.Normalize())
			tangent[faceEdges[0]] = circularTangent(newTangent, edge0)
			tangent[faceEdges[1]] = circularTangent(newTangent.Scale(-1), edge1)
			// Fix these tangents to keep them even to the edges.
			fixedHalfedge[faceEdges[0]] = true
			fixedHalfedge[faceEdges[1]] = true
		}
	}

	ts.distributeTangents(tangent, fixedHalfedge)

	// halfedgeTangent_ = move(tangent): flatten to the bridge's 4-per layout.
	flat := make([]float64, 4*numHalfedge)
	for i, t := range tangent {
		flat[4*i] = t.X
		flat[4*i+1] = t.Y
		flat[4*i+2] = t.Z
		flat[4*i+3] = t.W
	}
	mi.h.SetHalfedgeTangents(flat)
}
