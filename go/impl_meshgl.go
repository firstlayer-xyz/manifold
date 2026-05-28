package manifold

import (
	"math"
	"sort"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// getMeshGLImpl is the Go port of the C++ template free function
// GetMeshGLImpl<Precision, I> (src/impl.h). Parametric over output
// precision (float32/float64) and index width (uint32/uint64) — the
// two C++ instantiations:
//
//	GetMeshGLImpl<float, uint32_t>   → getMeshGLImpl[float32, uint32]
//	GetMeshGLImpl<double, uint64_t>  → getMeshGLImpl[float64, uint64]
//
// The C++ derives whether to apply the FLT_EPSILON tolerance floor
// via `std::is_same<Precision, float>::value`; Go does the same via
// a runtime type assertion on a zero value of P.
func getMeshGLImpl[P float32 | float64, I uint32 | uint64](
	impl *Impl, normalIdx int,
) meshGLP[P, I] {
	// Mirror std::is_same<Precision, float>::value.
	var zeroP P
	_, useSingleTolerance := any(zeroP).(float32)
	s := impl.Scalars()
	numProp := s.NumProp
	numTri := impl.HalfedgeCount() / 3
	verts := impl.Verts()
	starts := impl.HalfedgeStarts()
	props := impl.HalfedgeProps()
	tangents := impl.HalfedgeTangents()
	triRef := impl.TriRefs()
	relations := impl.MeshIDTransforms()
	properties := impl.Properties()

	numVert := len(verts) // Impl::NumVert
	numPropVert := numVert
	if numProp > 0 {
		numPropVert = len(properties) / numProp
	}

	isOriginal := s.OriginalID >= 0
	updateNormals := !isOriginal && normalIdx >= 0

	var out meshGLP[P, I]
	out.NumProp = 3 + numProp
	out.Tolerance = P(s.Tolerance)
	if useSingleTolerance {
		minB, maxB := impl.BBox()
		bBox := geom.Box{Min: minB, Max: maxB}
		floor := P(fltEpsilon * bBox.Scale())
		if floor > out.Tolerance {
			out.Tolerance = floor
		}
	}
	out.TriVerts = make([]I, 3*numTri)

	// halfedgeTangent_ → output halfedgeTangent (interleaved 4 floats per).
	numHalfedge := len(tangents) / 4
	out.HalfedgeTangent = make([]P, 4*numHalfedge)
	for i := 0; i < numHalfedge; i++ {
		out.HalfedgeTangent[4*i] = P(tangents[4*i])
		out.HalfedgeTangent[4*i+1] = P(tangents[4*i+1])
		out.HalfedgeTangent[4*i+2] = P(tangents[4*i+2])
		out.HalfedgeTangent[4*i+3] = P(tangents[4*i+3])
	}

	// Sort the triangles into runs: originals keep input order; others
	// stable-sort by (originalID, meshID).
	out.FaceID = make([]I, numTri)
	triNew2Old := make([]int, numTri)
	for i := range triNew2Old {
		triNew2Old[i] = i
	}
	if !isOriginal {
		sort.SliceStable(triNew2Old, func(a, b int) bool {
			ra := triRef[triNew2Old[a]]
			rb := triRef[triNew2Old[b]]
			if ra.OriginalID != rb.OriginalID {
				return ra.OriginalID < rb.OriginalID
			}
			return ra.MeshID < rb.MeshID
		})
	}

	// Build a meshID → Relation map for lookup. Also track which
	// relations have been consumed; leftovers get a trailing empty run.
	relByID := make(map[int32]bridge.MeshIDRelation, len(relations))
	for _, r := range relations {
		relByID[r.MeshID] = r
	}

	addRun := func(triIdx int, rel bridge.MeshIDRelation) {
		out.RunIndex = append(out.RunIndex, I(3*triIdx))
		out.RunOriginalID = append(out.RunOriginalID, uint32(rel.OriginalID))
		var flags uint8
		if rel.BackSide {
			flags |= 1
		}
		if rel.HasNormals {
			flags |= 2
		}
		out.RunFlags = append(out.RunFlags, flags)
		if !isOriginal {
			for col := 0; col < 4; col++ {
				for row := 0; row < 3; row++ {
					out.RunTransform = append(out.RunTransform, P(rel.Transform[col][row]))
				}
			}
		}
	}

	lastID := int32(-1)
	for tri := 0; tri < numTri; tri++ {
		oldTri := triNew2Old[tri]
		ref := triRef[oldTri]
		// faceID: ref.faceID if >= 0 else coplanarID
		var fid int32
		if ref.FaceID >= 0 {
			fid = ref.FaceID
		} else {
			fid = ref.CoplanarID
		}
		out.FaceID[tri] = I(fid)
		for i := 0; i < 3; i++ {
			out.TriVerts[3*tri+i] = I(starts[3*oldTri+i])
		}

		if ref.MeshID != lastID {
			rel := relByID[ref.MeshID]
			addRun(tri, rel)
			delete(relByID, ref.MeshID)
			lastID = ref.MeshID
		}
	}
	// Trailing runs for originals that contributed no faces (std::map
	// iteration order = ascending meshID; mirror that with a sort).
	leftoverIDs := make([]int32, 0, len(relByID))
	for id := range relByID {
		leftoverIDs = append(leftoverIDs, id)
	}
	sort.Slice(leftoverIDs, func(a, b int) bool { return leftoverIDs[a] < leftoverIDs[b] })
	for _, id := range leftoverIDs {
		addRun(numTri, relByID[id])
	}
	out.RunIndex = append(out.RunIndex, I(3*numTri))

	// Early return for no extra props: vertProperties is just positions.
	if numProp == 0 {
		out.VertProperties = make([]P, 3*numPropVert)
		for i := 0; i < numPropVert; i++ {
			v := verts[i]
			out.VertProperties[3*i] = P(v.X)
			out.VertProperties[3*i+1] = P(v.Y)
			out.VertProperties[3*i+2] = P(v.Z)
		}
		return out
	}

	// numProp > 0: duplicate verts that have different props attached
	// across triangles. vertPropPair[vert] is the (prop, outputIdx) bin.
	vert2idx := make([]int, numVert)
	for i := range vert2idx {
		vert2idx[i] = -1
	}
	type bin struct{ prop, idx int }
	vertPropPair := make([][]bin, numVert)
	out.VertProperties = make([]P, 0, numPropVert*out.NumProp)

	for runIdx := 0; runIdx+1 < len(out.RunIndex); runIdx++ {
		start := int(out.RunIndex[runIdx]) / 3
		end := int(out.RunIndex[runIdx+1]) / 3
		for tri := start; tri < end; tri++ {
			for i := 0; i < 3; i++ {
				prop := int(props[3*triNew2Old[tri]+i])
				vert := int(out.TriVerts[3*tri+i])

				bn := vertPropPair[vert]
				found := false
				for _, b := range bn {
					if b.prop == prop {
						out.TriVerts[3*tri+i] = I(b.idx)
						found = true
						break
					}
				}
				if found {
					continue
				}
				idx := len(out.VertProperties) / out.NumProp
				out.TriVerts[3*tri+i] = I(idx)
				vertPropPair[vert] = append(vertPropPair[vert], bin{prop: prop, idx: idx})

				v := verts[vert]
				out.VertProperties = append(out.VertProperties, P(v.X), P(v.Y), P(v.Z))
				for p := 0; p < numProp; p++ {
					out.VertProperties = append(out.VertProperties, P(properties[prop*numProp+p]))
				}

				if updateNormals {
					normal := geom.Vec3{
						X: float64(out.VertProperties[idx*out.NumProp+3+normalIdx+0]),
						Y: float64(out.VertProperties[idx*out.NumProp+3+normalIdx+1]),
						Z: float64(out.VertProperties[idx*out.NumProp+3+normalIdx+2]),
					}
					runHasN := !isOriginal && (out.RunFlags[runIdx]&2) != 0
					if !isOriginal && !runHasN {
						// Apply inverse-frame transform (with backSide sign).
						var m [4][3]float64
						for col := 0; col < 4; col++ {
							for row := 0; row < 3; row++ {
								m[col][row] = float64(out.RunTransform[runIdx*12+col*3+row])
							}
						}
						sign := 1.0
						if out.RunFlags[runIdx]&1 != 0 {
							sign = -1.0
						}
						normalT := geom.NormalTransform(geom.Mat3x4(m))
						normal = normalT.MulVec3(normal).Scale(sign)
					}
					// SafeNormalize: zero out NaN/inf, otherwise normalize.
					length := math.Sqrt(normal.Dot(normal))
					if length > 0 && !math.IsNaN(length) && !math.IsInf(length, 0) {
						normal = geom.Vec3{X: normal.X / length, Y: normal.Y / length, Z: normal.Z / length}
					} else {
						normal = geom.Vec3{}
					}
					out.VertProperties[idx*out.NumProp+3+normalIdx+0] = P(normal.X)
					out.VertProperties[idx*out.NumProp+3+normalIdx+1] = P(normal.Y)
					out.VertProperties[idx*out.NumProp+3+normalIdx+2] = P(normal.Z)
				}

				if vert2idx[vert] == -1 {
					vert2idx[vert] = idx
				} else {
					out.MergeFromVert = append(out.MergeFromVert, I(idx))
					out.MergeToVert = append(out.MergeToVert, I(vert2idx[vert]))
				}
			}
		}
	}
	return out
}

// meshGLP is the Go mirror of C++ MeshGLP<Precision, I> — a generic
// MeshGL struct parametric over precision and index width.
type meshGLP[P float32 | float64, I uint32 | uint64] struct {
	NumProp         int
	VertProperties  []P
	TriVerts        []I
	MergeFromVert   []I
	MergeToVert     []I
	RunIndex        []I
	RunOriginalID   []uint32
	RunTransform    []P
	RunFlags        []uint8
	FaceID          []I
	HalfedgeTangent []P
	Tolerance       P
}
