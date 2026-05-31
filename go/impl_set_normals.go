package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// SetNormals is the Go port of C++ Manifold::Impl::SetNormals
// (src/smoothing.cpp). For each vert, computes a per-vert normal and
// writes it into property slots [normalIdx, normalIdx+3). Edges
// sharper than minSharpAngle (degrees) split the vert into multiple
// pseudo-normals — one per "smooth region" delimited by sharp edges.
//
// Algorithm (mirrors C++):
//
//  1. Clamp minSharpAngle to kMinSharpAngle and count per-vert sharp
//     incidence (vertNumSharp). A vert with sharp count < 2 is a
//     "smooth vert" and takes a single normal; >= 2 means multi-normal.
//  2. Allocate a fresh properties_ buffer of size numProp*NumPropVert
//     (zeroed). Save the old buffer; numProp = max(oldNumProp, normalIdx+3).
//  3. Save the old halfedge_.propVert_ array and mark every entry -1
//     (the "unassigned" sentinel the main loop watches for).
//  4. For each startEdge in order, if its propVert is still -1, walk
//     the vertex fan and either (smooth vert) write the single normal
//     once, or (multi-normal) accumulate per-group pseudo-normals via
//     the two-arg ForVert and split propVerts at sharp edges.
//
// The non-zero normalIdx code path applies an inverse-normal-frame
// transform per meshID so the stored normal is in mesh-local frame —
// this matches the legacy MeshGL export contract.
func (mi *MutableImpl) SetNormals(normalIdx int, minSharpAngle float64) {
	if mi.HalfedgeCount() == 0 {
		return
	}
	if normalIdx < 0 {
		return
	}
	if minSharpAngle < kMinSharpAngle {
		minSharpAngle = kMinSharpAngle
	}

	starts := append([]int32(nil), mi.HalfedgeStarts()...)
	pairs := append([]int32(nil), mi.HalfedgePairs()...)
	sourceProps := mi.HalfedgeProps()
	faceNormals := mi.FaceNormals()
	vertNormals := mi.VertNormals()
	vertPos := mi.Verts()
	triRefs := mi.TriRefs()
	rels := mi.MeshIDTransforms()
	numHalfedge := len(starts)
	numVert := len(vertPos)
	oldNumProp := mi.NumProp()

	// Pass 1: count sharp incidence.
	vertNumSharp := make([]int32, numVert)
	for e := 0; e < numHalfedge; e++ {
		s := starts[e]
		eEnd := starts[nextHalfedge(e)]
		if s >= eEnd {
			continue // not forward; the pair will be processed once
		}
		pair := int(pairs[e])
		dihedral := geom.Degrees(geom.AngleBetween(faceNormals[e/3], faceNormals[pair/3]))
		if dihedral > minSharpAngle {
			vertNumSharp[s]++
			vertNumSharp[eEnd]++
		}
	}

	// Pass 2: resize properties.
	numProp := oldNumProp
	if normalIdx+3 > numProp {
		numProp = normalIdx + 3
	}
	oldProperties := append([]float64(nil), mi.Properties()...)
	numPropVert := numVert
	if oldNumProp > 0 {
		numPropVert = len(oldProperties) / oldNumProp
	}
	// Properties grows dynamically below; start with the same propVert
	// count, all-zero. The C++ swap+resize is the same effect.
	props := make([]float64, numProp*numPropVert)

	// Pass 3: snapshot the existing halfedge propVerts AND set every
	// slot in the working buffer to -1 in a SINGLE parallel pass,
	// matching C++ at src/smoothing.cpp:489-493:
	//   for_each_n(autoPolicy(halfedge_.size(), 1e5), countAt(0),
	//              halfedge_.size(), [&](int i) {
	//                oldHalfedgeProp[i] = halfedge_.Prop(i);
	//                halfedge_.SetProp(i, -1);
	//              });
	oldHalfedgeProp := make([]int32, numHalfedge)
	halfedgeProps := make([]int32, numHalfedge)
	parallel.ForEachN(parallel.AutoPolicy(numHalfedge, 100000), numHalfedge, func(i int) {
		oldHalfedgeProp[i] = sourceProps[i]
		halfedgeProps[i] = -1
	})

	// Lazy per-meshID inverse-normal-transform cache (used only when
	// normalIdx != 0). meshIdTransform maps meshID → 3x3 matrix.
	relByID := make(map[int]geom.Mat3, len(rels))
	for _, r := range rels {
		relByID[r.MeshID] = inverseNormalTransformFor(r)
	}
	getTransform := func(meshID int) geom.Mat3 { return relByID[meshID] }

	// Pass 4: per-vertex assignment.
	for startEdge := 0; startEdge < numHalfedge; startEdge++ {
		if halfedgeProps[startEdge] >= 0 {
			continue
		}
		vert := int(starts[startEdge])

		if vertNumSharp[vert] < 2 {
			// Smooth vert: single normal across the entire fan.
			worldNormal := vertNormals[vert]
			normal := worldNormal
			if normalIdx != 0 {
				normal = getTransform(triRefs[startEdge/3].MeshID).MulVec3(worldNormal)
			}
			var lastProp int32 = -1
			forVert(startEdge, pairs, func(current int) {
				prop := oldHalfedgeProp[current]
				halfedgeProps[current] = prop
				if prop == lastProp {
					return
				}
				lastProp = prop
				// Copy old properties, then overwrite normal slot.
				for p := 0; p < oldNumProp; p++ {
					props[int(prop)*numProp+p] = oldProperties[int(prop)*oldNumProp+p]
				}
				props[int(prop)*numProp+normalIdx+0] = normal.X
				props[int(prop)*numProp+normalIdx+1] = normal.Y
				props[int(prop)*numProp+normalIdx+2] = normal.Z
			})
			continue
		}

		// Multi-normal vert: walk around once to find a sharp-edge
		// boundary; then compute per-group pseudo-normals via the
		// two-arg ForVert.
		centerPos := vertPos[vert]

		// Find a sharp edge to start on. Walks current=NextHalfedge(Pair(current))
		// until a dihedral > minSharpAngle is found (or we loop back).
		current := startEdge
		prevFace := current / 3
		for {
			next := nextHalfedge(int(pairs[current]))
			face := next / 3
			dihedral := geom.Degrees(geom.AngleBetween(faceNormals[face], faceNormals[prevFace]))
			if dihedral > minSharpAngle {
				break
			}
			current = next
			prevFace = face
			if current == startEdge {
				break
			}
		}
		endEdge := current

		// FaceEdge is the per-edge transform value for the two-arg ForVert:
		// the face id and the unit-vector from centerPos to End(edge).
		type faceEdge struct {
			face           int
			normalizedEdge geom.Vec3
		}
		var groups []int
		var normals []geom.Vec3
		var meshIds []int

		forVertTransform[faceEdge](endEdge, pairs,
			func(edge int) faceEdge {
				endVert := starts[nextHalfedge(edge)]
				return faceEdge{
					face:           edge / 3,
					normalizedEdge: vertPos[endVert].Sub(centerPos).SafeNormalize(),
				}
			},
			func(edge int, here faceEdge, next *faceEdge) {
				dihedral := geom.Degrees(geom.AngleBetween(faceNormals[here.face], faceNormals[next.face]))
				if dihedral > minSharpAngle {
					normals = append(normals, geom.Vec3{})
					meshIds = append(meshIds, triRefs[next.face].MeshID)
				}
				groups = append(groups, len(normals)-1)
				if next.normalizedEdge.IsFinite() {
					dir := next.normalizedEdge.Cross(here.normalizedEdge).SafeNormalize()
					phi := geom.AngleBetween(here.normalizedEdge, next.normalizedEdge)
					last := len(normals) - 1
					normals[last] = normals[last].Add(geom.Vec3{
						X: dir.X * phi, Y: dir.Y * phi, Z: dir.Z * phi,
					})
				} else {
					// Propagate the previous normalizedEdge so the next
					// iteration's `here` has a finite value to work with.
					next.normalizedEdge = here.normalizedEdge
				}
			})

		// Apply per-mesh inverse-normal-frame and SafeNormalize.
		for i, n := range normals {
			if normalIdx != 0 {
				n = getTransform(meshIds[i]).MulVec3(n)
			}
			normals[i] = n.SafeNormalize()
		}

		// Assign propVerts for each halfedge around the fan, splitting
		// when the group changes mid-shared-propVert.
		lastGroup := 0
		var lastProp int32 = -1
		var newProp int32 = -1
		idx := 0
		forVert(endEdge, pairs, func(current1 int) {
			prop := oldHalfedgeProp[current1]
			if groups[idx] != lastGroup && groups[idx] != 0 && prop == lastProp {
				// Split: duplicate propVert with the new normal.
				lastGroup = groups[idx]
				newProp = int32(len(props) / numProp)
				oldRow := int(prop) * oldNumProp
				newRow := int(newProp) * numProp
				// Extend props by numProp slots.
				props = append(props, make([]float64, numProp)...)
				for p := 0; p < oldNumProp; p++ {
					props[newRow+p] = oldProperties[oldRow+p]
				}
				props[newRow+normalIdx+0] = normals[groups[idx]].X
				props[newRow+normalIdx+1] = normals[groups[idx]].Y
				props[newRow+normalIdx+2] = normals[groups[idx]].Z
			} else if prop != lastProp {
				lastProp = prop
				newProp = prop
				oldRow := int(prop) * oldNumProp
				newRow := int(prop) * numProp
				for p := 0; p < oldNumProp; p++ {
					props[newRow+p] = oldProperties[oldRow+p]
				}
				props[newRow+normalIdx+0] = normals[groups[idx]].X
				props[newRow+normalIdx+1] = normals[groups[idx]].Y
				props[newRow+normalIdx+2] = normals[groups[idx]].Z
			}
			halfedgeProps[current1] = newProp
			idx++
		})
	}

	// Commit.
	mi.SetNumProp(numProp)
	mi.SetProperties(props)
	mi.SetHalfedgesRaw(starts, halfedgeProps, pairs)
}

// inverseNormalTransformFor returns Relation.GetInverseNormalTransform()
// for a meshIDRelation: InverseNormalTransform(transform) *
// (backSide ? -1 : 1). Mirrors the C++ Relation::GetInverseNormalTransform.
func inverseNormalTransformFor(r meshIDRelation) geom.Mat3 {
	m := geom.InverseNormalTransform(r.Transform)
	if r.BackSide {
		for col := 0; col < 3; col++ {
			for row := 0; row < 3; row++ {
				m[col][row] = -m[col][row]
			}
		}
	}
	return m
}
